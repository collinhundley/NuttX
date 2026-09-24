/****************************************************************************
 * include/nuttx/serial/timed_halfduplex.h
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

#ifndef __INCLUDE_NUTTX_SERIAL_TIMED_HALFDUPLEX_H
#define __INCLUDE_NUTTX_SERIAL_TIMED_HALFDUPLEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Opt-in, exclusive-port packet transport.  Ordinary serial users retain
 * their existing behavior.  All sequence ranges are half-open.  A gap
 * denotes discarded input or ambiguous DMA continuity; reset parsers and
 * any frame-period qualification on that gap.
 */

#define TIOCSTIMEDHDX       _TIOC(0x0038)
#define TIOCGRXTIMEDHDX     _TIOC(0x0039)
#define TIOCSTXTIMEDHDX     _TIOC(0x003a)
#define TIOCGTIMEDHDX       _TIOC(0x003b)
#define SERIAL_THDX_MAX_PACKET 80

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct serial_thdx_config_s
{
  uint64_t (*clock_us)(void); /* IRQ-safe, monotonic microsecond clock */
  bool enable;
};

struct serial_thdx_status_s
{
  uint64_t first_sequence;
  uint64_t end_sequence;
  uint64_t start_lower_bound_us;
  uint64_t idle_observed_us; /* Zero until the UART IDLE interrupt */
  uint64_t tx_complete_us;   /* USART TC, not DMA completion */
  uint32_t rx_overruns;
  bool timing_valid;
  bool tx_busy;
  bool tx_fault;
};

struct serial_thdx_read_s
{
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  uint64_t first_sequence;
  uint64_t end_sequence;
  struct serial_thdx_status_s status;
};

struct serial_thdx_tx_s
{
  const uint8_t *buffer;
  uint16_t length;
  uint64_t request_first_sequence;
  uint64_t request_end_sequence;
  uint32_t frame_period_us;
  uint32_t max_idle_age_us; /* Cap from conservative request end and IDLE */
  uint32_t guard_us;
};

#endif /* __INCLUDE_NUTTX_SERIAL_TIMED_HALFDUPLEX_H */
