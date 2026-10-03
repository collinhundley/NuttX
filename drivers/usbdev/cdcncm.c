/****************************************************************************
 * drivers/usbdev/cdcncm.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* USB CDC Network Control Model, revision 1.0, NTB16 only. */

/* Adapted from Apache NuttX cdcncm.c, commit
 * 4e79741e7db15b74b9df78219b8c9e3927689410.  This backport deliberately uses
 * the existing net_driver_s API, NTB16, one TX request and no aggregation.
 * Later upstream fixes for EP0 ownership, notification ownership, immediate
 * TX and reconnect handling are incorporated without the lower-half API.
 */

#include <nuttx/config.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/ip.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcncm.h>
#include <nuttx/wqueue.h>
#include "cdcncm_ntb.h"

#if CONFIG_SCHED_LPNTHREADS != 1
#  error "The NCM backport requires one LPWORK thread for teardown ordering"
#endif

#define CDCNCM_CONFIGID 1
#define CDCNCM_VERSIONNO 0x0100
#define CDCNCM_NINTERFACES 2
#define CDCNCM_MACSTRID 1
#define CDCNCM_MXDESCLEN 128
#define CDC_SUBCLASS_NCM 0x0d
#define CDC_DSUBTYPE_NCM 0x1a
#define CDC_DATA_PROTO_NCMNTB 1
#define SIZEOF_NCM_FUNCDESC 6
#define NCAPS 1 /* Ethernet packet filter requests; no optional CRC/NTB32. */
#define CONFIG_CDCNCM_EPINTIN_FSSIZE 16
#define CONFIG_CDCNCM_EPBULKIN_FSSIZE 64
#define CONFIG_CDCNCM_EPBULKOUT_FSSIZE 64
#define NCM_GET_NTB_PARAMETERS 0x80
#define NCM_GET_NTB_FORMAT 0x83
#define NCM_SET_NTB_FORMAT 0x84
#define NCM_GET_NTB_INPUT_SIZE 0x85
#define NCM_SET_NTB_INPUT_SIZE 0x86

struct cdc_ncm_funcdesc_s
{
  uint8_t size;
  uint8_t type;
  uint8_t subtype;
  uint8_t version[2];
  uint8_t netcaps;
};

struct cdcncm_driver_s
{
  struct usbdevclass_driver_s usbdev;
  struct usbdev_devinfo_s devinfo;
  struct net_driver_s dev;
  struct work_s work;
  struct usbdev_req_s *ctrlreq;
  struct usbdev_req_s *notifyreq;
  struct usbdev_req_s *rdreq;
  struct usbdev_req_s *wrreq;
  struct usbdev_ep_s *epint;
  struct usbdev_ep_s *epbulkin;
  struct usbdev_ep_s *epbulkout;
  uint16_t pktbuf[(NCM_FRAME_SIZE + CONFIG_NET_GUARDSIZE + 1) / 2];
  struct ncm_frame_s frames[NCM_MAX_DATAGRAMS];
  uint32_t generation;
  uint32_t rxgeneration;
  uint32_t workgeneration;
  uint32_t replygeneration;
  uint16_t sequence;
  uint16_t inputsize;
  uint16_t filter;
  uint8_t config;
  uint8_t alt;
  uint8_t notify;
  unsigned frame;
  int nframes;
  bool registered;
  bool stopping;
  bool suspended;
  bool rxqueued;
  bool rxpending;
  bool txbusy;
  bool replypending;
  bool notifybusy;
};

/* Single instance. Set by board code before composite_initialize(). */

static uint8_t g_devmac[6];
static char g_hostmac[13];

static void cdcncm_work(void *arg);
static void cdcncm_mkepdesc(int epidx, struct usb_epdesc_s *epdesc,
                            struct usbdev_devinfo_s *devinfo, bool hispeed);
static void cdcncm_reset(struct cdcncm_driver_s *self);

static bool cdcncm_active(struct cdcncm_driver_s *self)
{
  return !self->stopping && self->config == 1 && self->alt == 1 &&
         self->filter != 0 && !self->suspended;
}

static void cdcncm_schedule(struct cdcncm_driver_s *self)
{
  irqstate_t flags = enter_critical_section();
  if (!self->stopping && work_available(&self->work))
    {
      work_queue(LPWORK, &self->work, cdcncm_work, self, 0);
    }
  leave_critical_section(flags);
}

static struct usbdev_req_s *cdcncm_allocreq(struct usbdev_ep_s *ep,
    unsigned size)
{
  struct usbdev_req_s *req = EP_ALLOCREQ(ep);
  if (req)
    {
      req->buf = EP_ALLOCBUFFER(ep, size);
      if (!req->buf)
        {
          EP_FREEREQ(ep, req);
          return NULL;
        }
      req->len = size;
    }
  return req;
}

static void cdcncm_freereq(struct usbdev_ep_s *ep,
                           struct usbdev_req_s **req)
{
  if (*req)
    {
      EP_FREEBUFFER(ep, (*req)->buf);
      EP_FREEREQ(ep, *req);
      *req = NULL;
    }
}

static void cdcncm_complete(struct usbdev_ep_s *ep,
                            struct usbdev_req_s *req)
{
  struct cdcncm_driver_s *self = ep->priv;
  irqstate_t flags = enter_critical_section();
  if (req == self->wrreq)
    {
      self->txbusy = false;
    }
  else if (req == self->notifyreq)
    {
      self->notifybusy = false;
    }
  else if (req == self->rdreq)
    {
      self->rxqueued = false;
      if (req->result == OK && cdcncm_active(self))
        {
          self->rxpending = true;
          self->rxgeneration = self->generation;
          self->nframes = -1;
          self->frame = 0;
        }
    }
  if (req->result != -ESHUTDOWN)
    {
      cdcncm_schedule(self);
    }
  leave_critical_section(flags);
}

static void cdcncm_ep0complete(struct usbdev_ep_s *ep,
                               struct usbdev_req_s *req)
{
}

/* No blocking waits: the single TX buffer is owned by USB until completion. */

static bool cdcncm_transmit(struct cdcncm_driver_s *self)
{
  irqstate_t flags = enter_critical_section();
  int ret = -ENETDOWN;
  if (!self->txbusy && cdcncm_active(self) &&
      self->workgeneration == self->generation)
    {
      self->wrreq->len = ncm_encode(self->wrreq->buf, self->dev.d_buf,
                                    self->dev.d_len, self->sequence++);
      if (self->wrreq->len != 0 && self->wrreq->len <= self->inputsize)
        {
          self->txbusy = true;
          self->wrreq->flags = USBDEV_REQFLAGS_NULLPKT;
          ret = EP_SUBMIT(self->epbulkin, self->wrreq);
          if (ret < 0)
            {
              self->txbusy = false;
            }
        }
    }
  leave_critical_section(flags);
  return ret == OK;
}

static int cdcncm_txpoll(struct net_driver_s *dev)
{
  struct cdcncm_driver_s *self = dev->d_private;
  if (dev->d_len > 0)
    {
      arp_out(dev);
      if (!devif_loopback(dev))
        {
          cdcncm_transmit(self);
          return 1;
        }
    }
  return 0;
}

static void cdcncm_notify(struct cdcncm_driver_s *self)
{
  uint8_t *buf = self->notifyreq->buf;
  irqstate_t flags = enter_critical_section();
  if (self->config != 0 && self->notify && !self->notifybusy &&
      !self->suspended)
    {
      memset(buf, 0, 16);
      buf[0] = 0xa1;
      ncm_put16(buf + 4, self->devinfo.ifnobase);
      if (self->notify == 2)
        {
          buf[1] = 0x2a; /* CONNECTION_SPEED_CHANGE */
          ncm_put16(buf + 6, 8);
          ncm_put32(buf + 8, 12000000);
          ncm_put32(buf + 12, 12000000);
          self->notifyreq->len = 16;
        }
      else
        {
          buf[1] = 0; /* NETWORK_CONNECTION */
          ncm_put16(buf + 2, cdcncm_active(self));
          self->notifyreq->len = 8;
        }
      self->notifybusy = true;
      if (EP_SUBMIT(self->epint, self->notifyreq) == OK)
        {
          self->notify--;
        }
      else
        {
          self->notifybusy = false;
        }
    }
  leave_critical_section(flags);
}

static void cdcncm_work(void *arg)
{
  struct cdcncm_driver_s *self = arg;
  irqstate_t flags;
  net_lock();
  if (self->stopping)
    {
      net_unlock();
      return;
    }

  if (cdcncm_active(self))
    {
      netdev_carrier_on(&self->dev);
    }
  else
    {
      netdev_carrier_off(&self->dev);
    }

  self->workgeneration = self->generation;
  if (self->replygeneration != self->generation)
    {
      self->replypending = false;
    }

  if (self->rxgeneration != self->generation)
    {
      self->rxpending = false;
      self->replypending = false;
    }

  if (!self->txbusy && cdcncm_active(self))
    {
      if (self->replypending)
        {
          self->replypending = !cdcncm_transmit(self);
        }

      if (self->rxpending && self->nframes < 0)
        {
          self->nframes = ncm_decode(self->rdreq->buf, self->rdreq->xfrd,
                                     self->frames);
          if (self->nframes < 0)
            {
              self->rxpending = false;
            }
        }

      while (self->rxpending && self->frame < (unsigned)self->nframes &&
             !self->txbusy && !self->replypending && cdcncm_active(self) &&
             self->rxgeneration == self->generation)
        {
          struct ncm_frame_s *frame = &self->frames[self->frame++];
          struct eth_hdr_s *eth = (struct eth_hdr_s *)self->dev.d_buf;
          memcpy(self->dev.d_buf, self->rdreq->buf + frame->offset,
                 frame->length);
          self->dev.d_len = frame->length;
          if (eth->type == HTONS(ETHTYPE_IP))
            {
              arp_ipin(&self->dev);
              ipv4_input(&self->dev);
              if (self->dev.d_len > 0)
                {
                  arp_out(&self->dev);
                }
            }
          else if (eth->type == HTONS(ETHTYPE_ARP))
            {
              arp_arpin(&self->dev);
            }
          else
            {
              self->dev.d_len = 0;
            }
          if (self->dev.d_len > 0)
            {
              self->replygeneration = self->workgeneration;
              self->replypending = !cdcncm_transmit(self);
            }
        }
      if (self->rxpending && self->frame == (unsigned)self->nframes)
        {
          self->rxpending = false;
        }
      if (!self->rxpending && !self->replypending && !self->txbusy &&
          IFF_IS_UP(self->dev.d_flags))
        {
          devif_poll(&self->dev, cdcncm_txpoll);
        }
    }

  flags = enter_critical_section();
  if (cdcncm_active(self) && !self->rxpending && !self->rxqueued)
    {
      self->rdreq->len = NCM_NTB_SIZE;
      self->rxqueued = true;
      if (EP_SUBMIT(self->epbulkout, self->rdreq) < 0)
        {
          self->rxqueued = false;
        }
    }
  leave_critical_section(flags);
  cdcncm_notify(self);
  net_unlock();
}

static int cdcncm_ifup(struct net_driver_s *dev)
{
  cdcncm_schedule(dev->d_private);
  return OK;
}

static int cdcncm_ifdown(struct net_driver_s *dev)
{
  return OK;
}

static int cdcncm_txavail(struct net_driver_s *dev)
{
  cdcncm_schedule(dev->d_private);
  return OK;
}

static void cdcncm_reset(struct cdcncm_driver_s *self)
{
  self->config = 0;
  self->alt = 0;
  self->filter = 0;
  self->suspended = false;
  self->notify = 0;
  self->sequence = 0;
  self->inputsize = NCM_NTB_SIZE;
  self->generation++;
  if (self->epint)
    EP_DISABLE(self->epint);
  if (self->epbulkin)
    EP_DISABLE(self->epbulkin);
  if (self->epbulkout)
    EP_DISABLE(self->epbulkout);
  cdcncm_schedule(self);
}

static int cdcncm_setconfig(struct cdcncm_driver_s *self, uint16_t config)
{
  struct usb_epdesc_s ep;
  int ret;
  if (config > 1)
    return -EINVAL;
  cdcncm_reset(self);
  if (config == 0)
    return OK;
  cdcncm_mkepdesc(CDCNCM_EP_INTIN_IDX, &ep, &self->devinfo, false);
  ret = EP_CONFIGURE(self->epint, &ep, true);
  if (ret == OK)
    self->config = 1;
  return ret;
}

static int cdcncm_setinterface(struct cdcncm_driver_s *self,
                               uint16_t interface, uint16_t alt)
{
  struct usb_epdesc_s ep;
  int ret;
  if (self->config != 1)
    return -EINVAL;
  if (interface == self->devinfo.ifnobase)
    return alt == 0 ? OK : -EINVAL;
  if (interface != self->devinfo.ifnobase + 1 || alt > 1)
    return -EINVAL;
  self->alt = 0;
  self->generation++;
  EP_DISABLE(self->epbulkin);
  EP_DISABLE(self->epbulkout);
  if (alt)
    {
      cdcncm_mkepdesc(CDCNCM_EP_BULKIN_IDX, &ep, &self->devinfo, false);
      ret = EP_CONFIGURE(self->epbulkin, &ep, false);
      if (ret < 0)
        return ret;
      cdcncm_mkepdesc(CDCNCM_EP_BULKOUT_IDX, &ep, &self->devinfo, false);
      ret = EP_CONFIGURE(self->epbulkout, &ep, true);
      if (ret < 0)
        {
          EP_DISABLE(self->epbulkin);
          return ret;
        }
      self->alt = 1;
    }
  self->notify = alt ? 2 : 1;
  cdcncm_schedule(self);
  return OK;
}

static int cdcncm_setup(struct usbdevclass_driver_s *driver,
                        struct usbdev_s *dev, const struct usb_ctrlreq_s *ctrl,
                        uint8_t *dataout, size_t outlen)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  uint16_t value = GETUINT16(ctrl->value);
  uint16_t index = GETUINT16(ctrl->index);
  uint16_t len = GETUINT16(ctrl->len);
  uint8_t *buf = self->ctrlreq->buf;
  int ret = -EOPNOTSUPP;

  if (ctrl->type == 0 && ctrl->req == USB_REQ_SETCONFIGURATION && len == 0)
    {
      ret = cdcncm_setconfig(self, value);
    }
  else if (ctrl->type == 0x01 && ctrl->req == USB_REQ_SETINTERFACE && len == 0)
    {
      ret = cdcncm_setinterface(self, index, value);
    }
  else if (ctrl->type == 0x81 && ctrl->req == USB_REQ_GETINTERFACE &&
           value == 0 && len == 1 && self->config == 1 &&
           (index == self->devinfo.ifnobase ||
            index == self->devinfo.ifnobase + 1))
    {
      buf[0] = index == self->devinfo.ifnobase ? 0 : self->alt;
      ret = 1;
    }
  else if (index == self->devinfo.ifnobase && self->config == 1)
    {
      if (ctrl->type == 0x21 && ctrl->req == ECM_SET_PACKET_FILTER &&
          len == 0 && (value & ~0x1f) == 0)
        {
          self->filter = value;
          self->notify = value ? 2 : 1;
          cdcncm_schedule(self);
          ret = 0;
        }
      else if (ctrl->type == 0xa1 && value == 0)
        {
          switch (ctrl->req)
            {
            case NCM_GET_NTB_PARAMETERS:
              memset(buf, 0, 28);
              ncm_put16(buf, 28);
              ncm_put16(buf + 2, 1); /* NTB16 only */
              ncm_put32(buf + 4, NCM_NTB_SIZE);
              ncm_put16(buf + 8, 4);
              ncm_put16(buf + 12, 4);
              ncm_put32(buf + 16, NCM_NTB_SIZE);
              ncm_put16(buf + 20, 4);
              ncm_put16(buf + 24, 4);
              ncm_put16(buf + 26, NCM_MAX_DATAGRAMS);
              ret = 28;
              break;
            case NCM_GET_NTB_FORMAT:
              ncm_put16(buf, 0);
              ret = 2;
              break;
            case NCM_GET_NTB_INPUT_SIZE:
              ncm_put32(buf, self->inputsize);
              ret = 4;
              break;
            }
        }
      else if (ctrl->type == 0x21 && value == 0 && self->alt == 0)
        {
          if (ctrl->req == NCM_SET_NTB_FORMAT && len == 0)
            {
              ret = 0;
            }
          else if (ctrl->req == NCM_SET_NTB_INPUT_SIZE && len == 4 &&
                   outlen == 4 && dataout &&
                   ncm_get32(dataout) >= NCM_NTB_SIZE)
            {
              /* Host capacity may exceed our advertised allocation. We
               * always send blocks <= 2048 bytes, satisfying either limit.
               */
              self->inputsize = NCM_NTB_SIZE;
              ret = 0;
            }
        }
    }

  if (ret >= 0)
    {
      self->ctrlreq->len = ret < len ? ret : len;
      self->ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      return composite_ep0submit(driver, dev, self->ctrlreq, ctrl);
    }
  return ret;
}

static void cdcncm_disconnect(struct usbdevclass_driver_s *driver,
                              struct usbdev_s *dev)
{
  cdcncm_reset((struct cdcncm_driver_s *)driver);
  DEV_CONNECT(dev);
}

static void cdcncm_suspend(struct usbdevclass_driver_s *driver,
                           struct usbdev_s *dev)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  self->suspended = true;
  cdcncm_schedule(self);
}

static void cdcncm_resume(struct usbdevclass_driver_s *driver,
                          struct usbdev_s *dev)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  self->suspended = false;
  self->notify = 2;
  cdcncm_schedule(self);
}

static void cdcncm_barrier(void *arg)
{
  nxsem_post(arg);
}

static void cdcncm_drain(struct cdcncm_driver_s *self)
{
  struct work_s barrier = {0};
  sem_t done;
  irqstate_t flags = enter_critical_section();
  self->stopping = true;
  work_cancel(LPWORK, &self->work);
  leave_critical_section(flags);

  /* Old NuttX has no work_cancel_sync. On this single LPWORK queue, a
   * barrier also drains a callback already dequeued before cancellation.
   * This is called from task context, never from LPWORK or an interrupt.
   */
  nxsem_init(&done, 0, 0);
  work_queue(LPWORK, &barrier, cdcncm_barrier, &done, 0);
  nxsem_wait_uninterruptible(&done);
  nxsem_destroy(&done);
}

static void cdcncm_unbind(struct usbdevclass_driver_s *driver,
                          struct usbdev_s *dev)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  /* composite_unbind holds the controller critical section. Drain work in
   * phase-one uninitialize (or bind's failure path), never in this callback.
   * A failure in an earlier composite member can also unbind this object
   * before bind was called: prevent reset from queuing work on absent EPs.
   */
  self->stopping = true;
  cdcncm_reset(self);
  cdcncm_freereq(dev->ep0, &self->ctrlreq);
  cdcncm_freereq(self->epint, &self->notifyreq);
  cdcncm_freereq(self->epbulkin, &self->wrreq);
  cdcncm_freereq(self->epbulkout, &self->rdreq);
  if (self->epint)
    DEV_FREEEP(dev, self->epint);
  if (self->epbulkin)
    DEV_FREEEP(dev, self->epbulkin);
  if (self->epbulkout)
    DEV_FREEEP(dev, self->epbulkout);
  self->epint = self->epbulkin = self->epbulkout = NULL;
}

static int cdcncm_bind(struct usbdevclass_driver_s *driver,
                       struct usbdev_s *dev)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  self->ctrlreq = cdcncm_allocreq(dev->ep0, 128);
  self->epint = DEV_ALLOCEP(dev, self->devinfo.epno[0], true, USB_EP_ATTR_XFER_INT);
  self->epbulkin = DEV_ALLOCEP(dev, self->devinfo.epno[1], true, USB_EP_ATTR_XFER_BULK);
  self->epbulkout = DEV_ALLOCEP(dev, self->devinfo.epno[2], false, USB_EP_ATTR_XFER_BULK);
  if (!self->ctrlreq || !self->epint || !self->epbulkin || !self->epbulkout)
    {
      goto fail;
    }
  self->epint->priv = self->epbulkin->priv = self->epbulkout->priv = self;
  self->notifyreq = cdcncm_allocreq(self->epint, 16);
  self->wrreq = cdcncm_allocreq(self->epbulkin, NCM_NTB_SIZE);
  self->rdreq = cdcncm_allocreq(self->epbulkout, NCM_NTB_SIZE);
  if (!self->notifyreq || !self->wrreq || !self->rdreq)
    goto fail;
  self->ctrlreq->callback = cdcncm_ep0complete;
  self->notifyreq->callback = self->wrreq->callback = self->rdreq->callback = cdcncm_complete;
  return OK;
fail:
  cdcncm_drain(self);
  cdcncm_unbind(driver, dev);
  return -ENOMEM;
}

static const struct usbdevclass_driverops_s g_usbdevops =
{
  cdcncm_bind, cdcncm_unbind, cdcncm_setup, cdcncm_disconnect,
  cdcncm_suspend, cdcncm_resume
};

static int cdcncm_classobject(int minor, struct usbdev_devinfo_s *info,
                              struct usbdevclass_driver_s **out)
{
  struct cdcncm_driver_s *self = kmm_zalloc(sizeof(*self));
  int ret;
  if (!self)
    return -ENOMEM;
  self->usbdev.ops = &g_usbdevops;
  self->usbdev.speed = USB_SPEED_FULL;
  self->devinfo = *info;
  self->inputsize = NCM_NTB_SIZE;
  self->dev.d_buf = (uint8_t *)self->pktbuf;
  self->dev.d_private = self;
  self->dev.d_ifup = cdcncm_ifup;
  self->dev.d_ifdown = cdcncm_ifdown;
  self->dev.d_txavail = cdcncm_txavail;
  memcpy(self->dev.d_mac.ether.ether_addr_octet, g_devmac, 6);
  ret = netdev_register(&self->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      kmm_free(self);
      return ret;
    }
  self->registered = true;
  *out = &self->usbdev;
  return OK;
}

static void cdcncm_uninitialize(struct usbdevclass_driver_s *driver)
{
  struct cdcncm_driver_s *self = (struct cdcncm_driver_s *)driver;
  if (self->registered)
    {
      cdcncm_drain(self);
      netdev_unregister(&self->dev);
      self->registered = false;
    }
  else
    {
      kmm_free(self);
    }
}

void cdcncm_setmac(const uint8_t device[6], const uint8_t host[6])
{
  memcpy(g_devmac, device, 6);
  snprintf(g_hostmac, sizeof(g_hostmac), "%02X%02X%02X%02X%02X%02X",
           host[0], host[1], host[2], host[3], host[4], host[5]);
}

static int cdcncm_mkstrdesc(uint8_t id, struct usb_strdesc_s *desc)
{
  unsigned i;
  if (id != CDCNCM_MACSTRID)
    return -EINVAL;
  desc->type = USB_DESC_TYPE_STRING;
  desc->len = 26;
  for (i = 0; i < 12; i++)
    {
      ((uint8_t *)(desc + 1))[i * 2] = g_hostmac[i];
      ((uint8_t *)(desc + 1))[i * 2 + 1] = 0;
    }
  return 26;
}

static void cdcncm_mkepdesc(int epidx, FAR struct usb_epdesc_s *epdesc,
                            FAR struct usbdev_devinfo_s *devinfo,
                            bool hispeed)
{
  uint16_t intin_mxpktsz   = CONFIG_CDCNCM_EPINTIN_FSSIZE;
  uint16_t bulkout_mxpktsz = CONFIG_CDCNCM_EPBULKOUT_FSSIZE;
  uint16_t bulkin_mxpktsz  = CONFIG_CDCNCM_EPBULKIN_FSSIZE;

  UNUSED(hispeed);

  epdesc->len  = USB_SIZEOF_EPDESC;      /* Descriptor length */
  epdesc->type = USB_DESC_TYPE_ENDPOINT; /* Descriptor type */

  switch (epidx)
    {
    case CDCNCM_EP_INTIN_IDX: /* Interrupt IN endpoint */
    {
      epdesc->addr            = USB_DIR_IN |
                                devinfo->epno[CDCNCM_EP_INTIN_IDX];
      epdesc->attr            = USB_EP_ATTR_XFER_INT;
      epdesc->mxpacketsize[0] = LSBYTE(intin_mxpktsz);
      epdesc->mxpacketsize[1] = MSBYTE(intin_mxpktsz);
      epdesc->interval        = 5;
    }
    break;

    case CDCNCM_EP_BULKIN_IDX:
    {
      epdesc->addr            = USB_DIR_IN |
                                devinfo->epno[CDCNCM_EP_BULKIN_IDX];
      epdesc->attr            = USB_EP_ATTR_XFER_BULK;
      epdesc->mxpacketsize[0] = LSBYTE(bulkin_mxpktsz);
      epdesc->mxpacketsize[1] = MSBYTE(bulkin_mxpktsz);
      epdesc->interval        = 0;
    }
    break;

    case CDCNCM_EP_BULKOUT_IDX:
    {
      epdesc->addr            = USB_DIR_OUT |
                                devinfo->epno[CDCNCM_EP_BULKOUT_IDX];
      epdesc->attr            = USB_EP_ATTR_XFER_BULK;
      epdesc->mxpacketsize[0] = LSBYTE(bulkout_mxpktsz);
      epdesc->mxpacketsize[1] = MSBYTE(bulkout_mxpktsz);
      epdesc->interval        = 0;
    }
    break;

    default:
      DEBUGPANIC();
    }
}
static int16_t cdcncm_mkcfgdesc(FAR uint8_t *desc,
                                FAR struct usbdev_devinfo_s *devinfo)
{
  int16_t len = 0;

#ifdef CONFIG_COMPOSITE_IAD
  /* Interface association descriptor */

  if (desc)
    {
      FAR struct usb_iaddesc_s *iaddesc;

      iaddesc = (FAR struct usb_iaddesc_s *)desc;
      iaddesc->len       = USB_SIZEOF_IADDESC;                  /* Descriptor length */
      iaddesc->type      = USB_DESC_TYPE_INTERFACEASSOCIATION;  /* Descriptor type */
      iaddesc->firstif   = devinfo->ifnobase;                   /* Number of first interface of the function */
      iaddesc->nifs      = devinfo->ninterfaces;                /* Number of interfaces associated with the function */
      iaddesc->classid   = USB_CLASS_CDC;                       /* Class code */
      iaddesc->subclass  = CDC_SUBCLASS_NCM;                    /* Sub-class code */
      iaddesc->protocol  = CDC_PROTO_NONE;                      /* Protocol code */
      iaddesc->ifunction = 0;                                   /* Index to string identifying the function */

      desc += USB_SIZEOF_IADDESC;
    }

  len += USB_SIZEOF_IADDESC;
#endif

  /* Communications Class Interface */

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase;
      ifdesc->alt      = 0;
      ifdesc->neps     = 1;
      ifdesc->classid  = USB_CLASS_CDC;
      ifdesc->subclass = CDC_SUBCLASS_NCM;
      ifdesc->protocol = CDC_PROTO_NONE;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct cdc_hdr_funcdesc_s *hdrdesc;

      hdrdesc = (FAR struct cdc_hdr_funcdesc_s *)desc;
      hdrdesc->size    = SIZEOF_HDR_FUNCDESC;
      hdrdesc->type    = USB_DESC_TYPE_CSINTERFACE;
      hdrdesc->subtype = CDC_DSUBTYPE_HDR;
      hdrdesc->cdc[0]  = LSBYTE(0x0120);
      hdrdesc->cdc[1]  = MSBYTE(0x0120);

      desc += SIZEOF_HDR_FUNCDESC;
    }

  len += SIZEOF_HDR_FUNCDESC;

  if (desc)
    {
      FAR struct cdc_union_funcdesc_s *uniondesc;

      uniondesc = (FAR struct cdc_union_funcdesc_s *)desc;
      uniondesc->size     = SIZEOF_UNION_FUNCDESC(1);
      uniondesc->type     = USB_DESC_TYPE_CSINTERFACE;
      uniondesc->subtype  = CDC_DSUBTYPE_UNION;
      uniondesc->master   = devinfo->ifnobase;
      uniondesc->slave[0] = devinfo->ifnobase + 1;

      desc += SIZEOF_UNION_FUNCDESC(1);
    }

  len += SIZEOF_UNION_FUNCDESC(1);

  if (desc)
    {
      FAR struct cdc_ecm_funcdesc_s *ecmdesc;

      ecmdesc = (FAR struct cdc_ecm_funcdesc_s *)desc;
      ecmdesc->size       = SIZEOF_ECM_FUNCDESC;
      ecmdesc->type       = USB_DESC_TYPE_CSINTERFACE;
      ecmdesc->subtype    = CDC_DSUBTYPE_ECM;
      ecmdesc->mac        = devinfo->strbase + CDCNCM_MACSTRID;
      ecmdesc->stats[0]   = 0;
      ecmdesc->stats[1]   = 0;
      ecmdesc->stats[2]   = 0;
      ecmdesc->stats[3]   = 0;
      ecmdesc->maxseg[0]  = LSBYTE(NCM_FRAME_SIZE);
      ecmdesc->maxseg[1]  = MSBYTE(NCM_FRAME_SIZE);
      ecmdesc->nmcflts[0] = LSBYTE(0);
      ecmdesc->nmcflts[1] = MSBYTE(0);
      ecmdesc->npwrflts   = 0;

      desc += SIZEOF_ECM_FUNCDESC;
    }

  len += SIZEOF_ECM_FUNCDESC;

  if (desc)
    {
      FAR struct cdc_ncm_funcdesc_s *ncmdesc;

      ncmdesc = (FAR struct cdc_ncm_funcdesc_s *)desc;
      ncmdesc->size       = SIZEOF_NCM_FUNCDESC;
      ncmdesc->type       = USB_DESC_TYPE_CSINTERFACE;
      ncmdesc->subtype    = CDC_DSUBTYPE_NCM;
      ncmdesc->version[0] = LSBYTE(CDCNCM_VERSIONNO);
      ncmdesc->version[1] = MSBYTE(CDCNCM_VERSIONNO);
      ncmdesc->netcaps    = NCAPS;

      desc += SIZEOF_NCM_FUNCDESC;
    }

  len += SIZEOF_NCM_FUNCDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_INTIN_IDX, epdesc, devinfo, false);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  /* Data Class Interface */

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase + 1;
      ifdesc->alt      = 0;
      ifdesc->neps     = 0;
      ifdesc->classid  = USB_CLASS_CDC_DATA;
      ifdesc->subclass = 0;
      ifdesc->protocol = CDC_DATA_PROTO_NCMNTB;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase + 1;
      ifdesc->alt      = 1;
      ifdesc->neps     = 2;
      ifdesc->classid  = USB_CLASS_CDC_DATA;
      ifdesc->subclass = 0;
      ifdesc->protocol = CDC_DATA_PROTO_NCMNTB;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_BULKIN_IDX, epdesc, devinfo, false);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_BULKOUT_IDX, epdesc, devinfo, false);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  DEBUGASSERT(len <= CDCNCM_MXDESCLEN);
  return len;
}

void cdcncm_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(*dev));
  dev->mkconfdesc = cdcncm_mkcfgdesc;
  dev->mkstrdesc = cdcncm_mkstrdesc;
  dev->classobject = cdcncm_classobject;
  dev->uninitialize = cdcncm_uninitialize;
  dev->nconfigs = 1;
  dev->configid = 1;
  dev->cfgdescsize = cdcncm_mkcfgdesc(NULL, NULL);
  dev->devinfo.ninterfaces = 2;
  dev->devinfo.nstrings = 1;
  dev->devinfo.nendpoints = 3;
}
