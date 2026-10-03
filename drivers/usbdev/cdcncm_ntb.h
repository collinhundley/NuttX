/* SPDX-License-Identifier: Apache-2.0 */
/* Bounded NCM 1.0 NTB16 framing; independent of the USB/network scheduler. */

#ifndef __DRIVERS_USBDEV_CDCNCM_NTB_H
#define __DRIVERS_USBDEV_CDCNCM_NTB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NCM_NTB_SIZE       2048
#define NCM_FRAME_SIZE     1514
#define NCM_MAX_DATAGRAMS  32
#define NCM_NTH_SIZE       12
/* NTH (12), NDP (16), then two padding bytes: IP starts at offset 44. */
#define NCM_TX_OFFSET      30

struct ncm_frame_s
{
  uint16_t offset;
  uint16_t length;
};

static inline uint16_t ncm_get16(const uint8_t *p)
{
  return p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t ncm_get32(const uint8_t *p)
{
  return ncm_get16(p) | ((uint32_t)ncm_get16(p + 2) << 16);
}

static inline void ncm_put16(uint8_t *p, uint16_t v)
{
  p[0] = v;
  p[1] = v >> 8;
}

static inline void ncm_put32(uint8_t *p, uint32_t v)
{
  ncm_put16(p, v);
  ncm_put16(p + 2, v >> 16);
}

static inline bool ncm_overlap(unsigned a, unsigned alen,
                               unsigned b, unsigned blen)
{
  return a < b + blen && b < a + alen;
}

/* Validate the complete block before passing any frame to the network.
 * Return a frame count, or -1 for malformed input.  Padding after wBlockLength
 * is allowed, but every descriptor and datagram must be within that length.
 */

static inline int ncm_decode(const uint8_t *buf, size_t received,
                             struct ncm_frame_s frames[NCM_MAX_DATAGRAMS])
{
  struct ncm_frame_s tables[NCM_MAX_DATAGRAMS];
  unsigned ntables = 0;
  unsigned count = 0;
  unsigned block;
  unsigned ndp;
  unsigned i;
  unsigned j;

  if (received < NCM_NTH_SIZE || received > NCM_NTB_SIZE ||
      ncm_get32(buf) != 0x484d434e || ncm_get16(buf + 4) != NCM_NTH_SIZE)
    {
      return -1;
    }

  block = ncm_get16(buf + 8);
  ndp = ncm_get16(buf + 10);
  if (block < NCM_NTH_SIZE || block > received || ndp == 0)
    {
      return -1;
    }

  while (ndp != 0)
    {
      unsigned len;
      bool terminated = false;

      if (ntables == NCM_MAX_DATAGRAMS || ndp < NCM_NTH_SIZE ||
          (ndp & 3) != 0 || ndp > block || block - ndp < 12 ||
          ncm_get32(buf + ndp) != 0x304d434e)
        {
          return -1;
        }

      len = ncm_get16(buf + ndp + 4);
      if (len < 12 || (len & 3) != 0 || len > block - ndp)
        {
          return -1;
        }

      for (i = 0; i < ntables; i++)
        {
          if (ncm_overlap(ndp, len, tables[i].offset, tables[i].length))
            {
              return -1; /* Also rejects cyclic NDP chains. */
            }
        }

      tables[ntables++] = (struct ncm_frame_s)
      {
        ndp, len
      };
      for (i = 8; i < len; i += 4)
        {
          unsigned offset = ncm_get16(buf + ndp + i);
          unsigned size = ncm_get16(buf + ndp + i + 2);
          if (offset == 0 && size == 0)
            {
              terminated = true;
              break;
            }

          if (count == NCM_MAX_DATAGRAMS || offset < NCM_NTH_SIZE ||
              ((offset + 14) & 3) != 0 || size < 14 || size > NCM_FRAME_SIZE ||
              offset > block || size > block - offset)
            {
              return -1;
            }

          frames[count++] = (struct ncm_frame_s)
          {
            offset, size
          };
        }

      if (!terminated)
        {
          return -1;
        }

      ndp = ncm_get16(buf + ndp + 6);
    }

  for (i = 0; i < count; i++)
    {
      for (j = 0; j < ntables; j++)
        {
          if (ncm_overlap(frames[i].offset, frames[i].length,
                          tables[j].offset, tables[j].length))
            {
              return -1;
            }
        }

      for (j = 0; j < i; j++)
        {
          if (ncm_overlap(frames[i].offset, frames[i].length,
                          frames[j].offset, frames[j].length))
            {
              return -1;
            }
        }
    }

  return count;
}

static inline size_t ncm_encode(uint8_t *buf, const uint8_t *frame,
                                size_t size, uint16_t sequence)
{
  if (size < 14 || size > NCM_FRAME_SIZE)
    {
      return 0;
    }

  memset(buf, 0, NCM_TX_OFFSET);
  ncm_put32(buf, 0x484d434e);
  ncm_put16(buf + 4, NCM_NTH_SIZE);
  ncm_put16(buf + 6, sequence);
  ncm_put16(buf + 8, NCM_TX_OFFSET + size);
  ncm_put16(buf + 10, NCM_NTH_SIZE);
  ncm_put32(buf + 12, 0x304d434e);
  ncm_put16(buf + 16, 16);
  ncm_put16(buf + 20, NCM_TX_OFFSET);
  ncm_put16(buf + 22, size);
  memcpy(buf + NCM_TX_OFFSET, frame, size);
  return NCM_TX_OFFSET + size;
}

#endif
