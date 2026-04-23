/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji KITAYAMA
 * Copyright (c) 2024, Brent Kowal (Analog Devices, Inc)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && defined(TUP_USBIP_MUSB)

#define MUSB_DEBUG 2
#define MUSB_REGS(rhport)   ((musb_regs_t*) MUSB_BASES[rhport])

#include "musb_type.h"
#include "device/dcd.h"

// Following symbols must be defined by port header
// - musb_dcd_int_enable/disable/clear/get_enable
// - musb_dcd_int_handler_enter/exit
#if defined(TUP_USBIP_MUSB_TI)
  #include "musb_ti.h"
#elif defined(TUP_USBIP_MUSB_ADI)
  #include "musb_max32.h"
#else
  #error "Unsupported MCU"
#endif

/*------------------------------------------------------------------
 * MACRO TYPEDEF CONSTANT ENUM DECLARATION
 *------------------------------------------------------------------*/

#define REQUEST_TYPE_INVALID  (0xFFu)

typedef union {
  volatile uint8_t   u8;
  volatile uint16_t  u16;
  volatile uint32_t  u32;
} hw_fifo_t;

typedef struct {
  union {
    uint8_t  *buf;      /* the start address of a transfer data buffer */
    tu_fifo_t *fifo;
  };
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */
  bool      armed;     /* true while a transfer is posted */
  bool      use_fifo;  /* true: buf is tu_fifo_t*; false: buf is plain byte pointer. */
} pipe_state_t;

// Pipe array layout (N = TUP_DCD_ENDPOINT_MAX):
//   [0] : EP0 (shared between IN/OUT control stages)
// One-direction-only IPs (CFG_TUD_ENDPOINT_ONE_DIRECTION_ONLY=1):
//   [1 .. n-1] : EP1..n-1 (single slot per endpoint)
// Bidirectional-capable IPs:
//   [1 .. N-1  ] : EP OUT
//   [N .. 2*N-2] : EP IN
#if CFG_TUD_ENDPOINT_ONE_DIRECTION_ONLY
  #define MUSB_PIPE_COUNT TUP_DCD_ENDPOINT_MAX
#else
  #define MUSB_PIPE_COUNT (2u * TUP_DCD_ENDPOINT_MAX - 1u)
#endif

typedef struct {
  union {
    tusb_control_request_t setup_packet;
    uint32_t setup_buffer[2];
  };
  uint16_t     remaining_ctrl; /* The number of bytes remaining in data stage of control transfer. */
  int8_t       status_out;
  pipe_state_t pipe[MUSB_PIPE_COUNT];
} dcd_data_t;

static dcd_data_t _dcd;

TU_ATTR_ALWAYS_INLINE static inline pipe_state_t* pipe_get(uint8_t epnum, tusb_dir_t epdir) {
#if CFG_TUD_ENDPOINT_ONE_DIRECTION_ONLY
  (void) epdir;
  return &_dcd.pipe[epnum];
#else
  if (epnum == 0) {
    return &_dcd.pipe[0];
  }
  size_t idx = epnum;
  if (epdir == TUSB_DIR_IN) {
    idx += TUP_DCD_ENDPOINT_MAX - 1u;
  }
  return &_dcd.pipe[idx];
#endif
}

//--------------------------------------------------------------------
// HW FIFO Helper
// Note: Index register is already set by caller
//--------------------------------------------------------------------

#if MUSB_CFG_DYNAMIC_FIFO

// musb is configured to use dynamic FIFO sizing.
// FF Size is encodded: 1 << (fifo_size[3:0] + 3) = 8 << fifo_size[3:0]
// FF Address is 8*ff_addr[12:0]
// First 64 bytes are reserved for EP0
static uint32_t alloced_fifo_bytes;

// ffsize is log2(mps) - 3 (round up)
TU_ATTR_ALWAYS_INLINE static inline uint8_t hwfifo_byte2size(uint16_t nbytes) {
  uint8_t ffsize = 28 - tu_min8(28, __builtin_clz(nbytes));
  if ((8u << ffsize) < nbytes) {
    ++ffsize;
  }
  return ffsize;
}

TU_ATTR_ALWAYS_INLINE static inline void hwfifo_reset(musb_regs_t* musb, unsigned epnum, unsigned is_rx) {
  (void) epnum;
  musb->fifo_size[is_rx] = 0;
  musb->fifo_addr[is_rx] = 0;
}

TU_ATTR_ALWAYS_INLINE static inline bool hwfifo_config(musb_regs_t* musb, unsigned epnum, unsigned is_rx, unsigned mps,
                                                       bool double_packet) {
  uint8_t ffsize = hwfifo_byte2size(mps);
  mps = 8 << ffsize; // round up to the next power of 2

  if (double_packet) {
    ffsize |= MUSB_FIFOSZ_DOUBLE_PACKET;
    mps <<= 1;
  }

  TU_ASSERT(alloced_fifo_bytes + mps <= MUSB_CFG_DYNAMIC_FIFO_SIZE);
  musb->fifo_addr[is_rx] = alloced_fifo_bytes / 8;
  musb->fifo_size[is_rx] = ffsize;

  volatile uint16_t* dp_disable = is_rx ? &musb->rx_doulbe_packet_disable : &musb->tx_double_packet_disable;
  if (double_packet) {
    *dp_disable &= ~(1u << epnum);
  } else {
    *dp_disable |= (1u << epnum);
  }

  alloced_fifo_bytes += mps;
  return true;
}

#else

TU_ATTR_ALWAYS_INLINE static inline void hwfifo_reset(musb_regs_t* musb, unsigned epnum, unsigned is_rx) {
  (void) musb; (void) epnum; (void) is_rx;
  // nothing to do for static FIFO
}

TU_ATTR_ALWAYS_INLINE static inline bool hwfifo_config(musb_regs_t* musb, unsigned epnum, unsigned is_rx, unsigned mps,
                                                       bool double_packet) {
  (void) mps;

  #if defined(TUP_USBIP_MUSB_ADI)
  // AnalogDevice FIFO sizes: EP1..7 = 512 B, EP8..9 = 2048 B, EP10..11 = 4096 B.
  // DPB requires FIFO >= 2 * MPS. For HS bulk (MPS=512) only EP >= 8 qualifies.
  // Force single-buffered on EP < 8 even if the caller requested DPB.
  if (epnum < 8 && (musb->power & MUSB_POWER_HSMODE)) {
    double_packet = false;
  }
  volatile uint8_t* csrh = &musb->indexed_csr.maxp_csr[is_rx].csrh;
  if (double_packet) {
    *csrh &= ~MUSB_CSRH_DISABLE_DOUBLE_PACKET;
  } else {
    *csrh |= MUSB_CSRH_DISABLE_DOUBLE_PACKET;
  }
  #else
  volatile uint16_t* dp_disable = is_rx ? &musb->rx_doulbe_packet_disable : &musb->tx_double_packet_disable;
  if (double_packet) {
    *dp_disable &= ~(1u << epnum);
  } else {
    *dp_disable |= (1u << epnum);
  }
  #endif

  return true;
}

#endif

// Flush FIFO and clear data toggle
TU_ATTR_ALWAYS_INLINE static inline void hwfifo_flush(musb_regs_t* musb, unsigned epnum, unsigned is_rx, bool clear_dtog) {
  (void) epnum;
  const uint8_t csrl_dtog = clear_dtog ? MUSB_CSRL_CLEAR_DATA_TOGGLE(is_rx) : 0;
  musb_ep_maxp_csr_t* maxp_csr = &musb->indexed_csr.maxp_csr[is_rx];
  // may need to flush twice for double packet
  for (unsigned i=0; i<2; i++) {
    if (maxp_csr->csrl & MUSB_CSRL_PACKET_READY(is_rx)) {
      maxp_csr->csrl = MUSB_CSRL_FLUSH_FIFO(is_rx) | csrl_dtog;
    }
  }
}

static void process_setup_packet(uint8_t rhport) {
  musb_regs_t* musb_regs = MUSB_REGS(rhport);

  // Read setup packet
  _dcd.setup_buffer[0] = musb_regs->fifo[0];
  _dcd.setup_buffer[1] = musb_regs->fifo[0];

  pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);
  pipe0->buf       = NULL;
  pipe0->length    = 0;
  pipe0->remaining = 0;
  dcd_event_setup_received(rhport, (const uint8_t*)(uintptr_t)&_dcd.setup_packet, true);

  const unsigned len    = _dcd.setup_packet.wLength;
  _dcd.remaining_ctrl   = len;
  const unsigned dir_in = tu_edpt_dir(_dcd.setup_packet.bmRequestType);
  /* Clear RX FIFO and reverse the transaction direction */
  if (len && dir_in) {
    musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, 0);
    ep_csr->csr0l = MUSB_CSRL0_RXRDYC;
  }
}

// write to txfifo using pipe_state_t info
static void pipe_write(musb_regs_t* musb_regs, pipe_state_t* pipe, uint8_t epnum) {
  musb_ep_csr_t* ep_csr = &musb_regs->indexed_csr;
  const uint16_t mps = ep_csr->tx_maxp & MUSB_TXMAXP_PACKET_SIZE_M;
  const uint16_t xact_len = tu_min16(mps, pipe->remaining);
  volatile void *hwfifo = &musb_regs->fifo[epnum];
  if (xact_len) {
    if (pipe->use_fifo) {
      tu_hwfifo_write_from_fifo(hwfifo, pipe->fifo, xact_len, NULL);
    } else {
      tu_hwfifo_write(hwfifo, pipe->buf, xact_len, NULL);
      pipe->buf += xact_len;
    }
    pipe->remaining -= xact_len;
  }
  ep_csr->tx_csrl = MUSB_TXCSRL1_TXRDY;
}

// Called from the TX interrupt. If the last queued packet finished the transfer,
// signal completion; otherwise queue the next packet.
static void process_epin(uint8_t rhport, musb_regs_t *musb_regs, uint8_t epnum) {
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, epnum);
  if (ep_csr->tx_csrl & MUSB_TXCSRL1_STALLED) {
    ep_csr->tx_csrl &= ~(MUSB_TXCSRL1_STALLED | MUSB_TXCSRL1_UNDRN);
    return; // sent STALL, do nothing
  }

  pipe_state_t* pipe = pipe_get(epnum, TUSB_DIR_IN);
  if (pipe->remaining > 0) {
    pipe_write(musb_regs, pipe, epnum);
  } else {
    // All bytes have been loaded into the FIFO. With double-packet buffering a
    // second packet may still be waiting in the FIFO when this IRQ fires (the
    // hardware signals TXRDY clear as soon as a slot frees, not when the wire
    // transfer finishes). Defer completion until FIFONE == 0 so we don't emit
    // a duplicate xfer_complete before the final packet has been sent.
    if (ep_csr->tx_csrl & MUSB_TXCSRL1_FIFONE) {
      return;
    }
    const uint16_t xferred_len = pipe->length;
    pipe->buf = NULL;
    pipe->armed = false;
    dcd_event_xfer_complete(rhport, tu_edpt_addr(epnum, TUSB_DIR_IN), xferred_len, XFER_RESULT_SUCCESS, true);
  }
}

// Drain one packet from the Rx FIFO into pipe->buf/fifo, update pipe state, and
// release the FIFO slot by clearing RXRDY. return true if short packet
static bool pipe_read(musb_regs_t* musb_regs, pipe_state_t* pipe, uint8_t epnum) {
  musb_ep_csr_t* ep_csr = &musb_regs->indexed_csr; // index already set in process_epout()
  const uint16_t mps = ep_csr->rx_maxp & MUSB_RXMAXP_PACKET_SIZE_M;
  const uint16_t rx_count = ep_csr->rx_count;
  const uint16_t xact_len = tu_min16(tu_min16(pipe->remaining, mps), rx_count);
  volatile void *hwfifo = &musb_regs->fifo[epnum];
  if (xact_len) {
    if (pipe->use_fifo) {
      tu_hwfifo_read_to_fifo(hwfifo, pipe->fifo, xact_len, NULL);
    } else {
      tu_hwfifo_read(hwfifo, pipe->buf, xact_len, NULL);
      pipe->buf += xact_len;
    }
    pipe->remaining -= xact_len;
  }
  ep_csr->rx_csrl = 0; /* Clear RXRDY - release this FIFO slot */

  return (xact_len < mps);
}

static void process_epout(uint8_t rhport, musb_regs_t *musb_regs, uint8_t epnum, bool is_isr) {
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, epnum);
  if (ep_csr->rx_csrl & MUSB_RXCSRL1_STALLED) {
    ep_csr->rx_csrl &= ~(MUSB_RXCSRL1_STALLED | MUSB_RXCSRL1_OVER);
    return; // sent STALL, do nothing
  }

  // Fail gracefully. Spurious interrupt.
  if (!(ep_csr->rx_csrl & MUSB_RXCSRL1_RXRDY)) {
    return;
  }

  pipe_state_t *pipe = pipe_get(epnum, TUSB_DIR_OUT);
  if (!pipe->armed) {
    // Packet is already ACK'd by hardware and sitting in the Rx FIFO, but no transfer is
    // posted. Do NOT flush (per MUSB spec §3.3.11 FlushFIFO) - that would silently drop
    // acknowledged data. Mask this endpoint's Rx interrupt so the ISR stops re-firing;
    // the FIFO stays occupied so hardware NAKs further OUT tokens (natural backpressure).
    // The next dcd_edpt_xfer() on this endpoint will drain the staged packet.
    musb_regs->intr_rxen &= (uint16_t) ~TU_BIT(epnum);
    return;
  }

  const bool is_short = pipe_read(musb_regs, pipe, epnum);

  // Transfer completes on a short packet or when the rx buffer is filled.
  if (is_short || pipe->remaining == 0) {
    const uint16_t xferred_len = pipe->length - pipe->remaining;
    pipe->buf = NULL;
    pipe->armed = false;
    dcd_event_xfer_complete(rhport, epnum, xferred_len, XFER_RESULT_SUCCESS, is_isr);
  }
}

static bool edpt_n_xfer(uint8_t rhport, uint8_t ep_addr, void *buffer, uint16_t total_bytes, bool use_fifo, bool is_isr) {
  const uint8_t epnum  = tu_edpt_number(ep_addr);
  const unsigned dir_in = tu_edpt_dir(ep_addr);

  pipe_state_t *pipe = pipe_get(epnum, dir_in);
  if (use_fifo) {
    pipe->fifo = (tu_fifo_t *)buffer;
  } else {
    pipe->buf = (uint8_t *)buffer;
  }
  pipe->length    = total_bytes;
  pipe->remaining = total_bytes;
  pipe->use_fifo  = use_fifo;
  pipe->armed     = true;

  musb_regs_t   *musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t *ep_csr    = get_ep_csr(musb_regs, epnum);

  if (dir_in) {
    pipe_write(musb_regs, pipe, epnum);
  } else {
    // Re-enable Rx interrupt (may have been masked by the no-buffer path in process_epout)
    musb_regs->intr_rxen |= (uint16_t)TU_BIT(epnum);

    // Drain any packet staged in the Rx FIFO from a prior no-buffer interrupt.
    // process_epout() fires dcd_event_xfer_complete() itself if the drain completes.
    if (ep_csr->rx_csrl & MUSB_RXCSRL1_RXRDY) {
      process_epout(rhport, musb_regs, epnum, is_isr);
    }
  }
  return true;
}

static bool edpt0_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes, bool is_isr)
{
  (void)rhport;
  TU_ASSERT(total_bytes <= 64); /* Current implementation supports for only up to 64 bytes. */
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, 0);
  pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);
  const unsigned req = _dcd.setup_packet.bmRequestType;
  TU_ASSERT(req != REQUEST_TYPE_INVALID || total_bytes == 0);

  if (req == REQUEST_TYPE_INVALID || _dcd.status_out) {
    /* STATUS OUT stage.
     * MUSB controller automatically handles STATUS OUT packets without
     * software helps. We do not have to do anything. And STATUS stage
     * may have already finished and received the next setup packet
     * without calling this function, so we have no choice but to
     * invoke the callback function of status packet here. */
    // TU_LOG1(" STATUS OUT ep_csr->csr0l = %x\r\n", ep_csr->csr0l);
    _dcd.status_out = 0;
    if (req == REQUEST_TYPE_INVALID) {
      dcd_event_xfer_complete(rhport, ep_addr, total_bytes, XFER_RESULT_SUCCESS, is_isr);
    } else {
      /* The next setup packet has already been received, it aborts
       * invoking callback function to avoid confusing TUSB stack. */
      TU_LOG1("Drop CONTROL_STAGE_ACK\r\n");
    }
    return true;
  }
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  if (tu_edpt_dir(req) == dir_in) { /* DATA stage */
    TU_ASSERT(total_bytes <= _dcd.remaining_ctrl);
    const unsigned rem = _dcd.remaining_ctrl;
    const unsigned len = TU_MIN(TU_MIN(rem, 64), total_bytes);
    volatile void *fifo_ptr = &musb_regs->fifo[0];
    if (dir_in) {
      tu_hwfifo_write(fifo_ptr, buffer, len, NULL);

      pipe0->buf       = buffer + len;
      pipe0->length    = len;
      pipe0->remaining = 0;

      _dcd.remaining_ctrl  = rem - len;
      if ((len < 64) || (rem == len)) {
        _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID; /* Change to STATUS/SETUP stage */
        _dcd.status_out = 1;
        /* Flush TX FIFO and reverse the transaction direction. */
        ep_csr->csr0l = MUSB_CSRL0_TXRDY | MUSB_CSRL0_DATAEND;
      } else {
        ep_csr->csr0l = MUSB_CSRL0_TXRDY; /* Flush TX FIFO to return ACK. */
      }
    } else {
      pipe0->buf       = buffer;
      pipe0->length    = len;
      pipe0->remaining = len;
      ep_csr->csr0l = MUSB_CSRL0_RXRDYC; /* Clear RX FIFO to return ACK. */
    }
  } else if (dir_in) {
    pipe0->buf       = NULL;
    pipe0->length    = 0;
    pipe0->remaining = 0;
    /* Clear RX FIFO and reverse the transaction direction */
    ep_csr->csr0l = MUSB_CSRL0_RXRDYC | MUSB_CSRL0_DATAEND;
  }
  return true;
}

static void process_ep0(uint8_t rhport)
{
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, 0);
  pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);
  uint_fast8_t csrl = ep_csr->csr0l;

  // 21.1.5: endpoint 0 service routine as peripheral

  if (csrl & MUSB_CSRL0_STALLED) {
    /* Returned STALL packet to HOST. */
    ep_csr->csr0l = 0; /* Clear STALL */
    return;
  }

  unsigned req = _dcd.setup_packet.bmRequestType;
  if (csrl & MUSB_CSRL0_SETEND) {
    TU_LOG1("   ABORT by the next packets\r\n");
    ep_csr->csr0l = MUSB_CSRL0_SETENDC;
    if (req != REQUEST_TYPE_INVALID && pipe0->buf) {
      /* DATA stage was aborted by receiving STATUS or SETUP packet. */
      pipe0->buf = NULL;
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      dcd_event_xfer_complete(rhport,
                              req & TUSB_DIR_IN_MASK,
                              pipe0->length - pipe0->remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    req = REQUEST_TYPE_INVALID;
    if (!(csrl & MUSB_CSRL0_RXRDY)) return; /* Received SETUP packet */
  }

  if (csrl & MUSB_CSRL0_RXRDY) {
    /* Received SETUP or DATA OUT packet */
    if (req == REQUEST_TYPE_INVALID) {
      /* SETUP */
      TU_ASSERT(sizeof(tusb_control_request_t) == ep_csr->count0,);
      process_setup_packet(rhport);
      return;
    }
    if (pipe0->buf) {
      /* DATA OUT */
      const unsigned vld = ep_csr->count0;
      const unsigned rem = pipe0->remaining;
      const unsigned len = TU_MIN(TU_MIN(rem, 64), vld);
      volatile void *fifo_ptr = &musb_regs->fifo[0];
      tu_hwfifo_read(fifo_ptr, pipe0->buf, len, NULL);

      pipe0->remaining = rem - len;
      _dcd.remaining_ctrl -= len;

      pipe0->buf = NULL;
      dcd_event_xfer_complete(rhport,
                              tu_edpt_addr(0, TUSB_DIR_OUT),
                              pipe0->length - pipe0->remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    return;
  }

  /* When CSRL0 is zero, it means that completion of sending any length packet
   * or receiving a zero length packet. */
  if (req != REQUEST_TYPE_INVALID && !tu_edpt_dir(req)) {
    /* STATUS IN */
    if (*(const uint16_t*)(uintptr_t)&_dcd.setup_packet == 0x0500) {
      /* The address must be changed on completion of the control transfer. */
      musb_regs->faddr = (uint8_t)_dcd.setup_packet.wValue;
    }
    _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            pipe0->length - pipe0->remaining,
                            XFER_RESULT_SUCCESS, true);
    return;
  }
  if (pipe0->buf) {
    /* DATA IN */
    pipe0->buf = NULL;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            pipe0->length - pipe0->remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

// Upon BUS RESET is detected, hardware havs already done:
// faddr = 0, index = 0, flushes all ep fifos, clears all ep csr, enabled all ep interrupts
static void process_bus_reset(uint8_t rhport) {
  musb_regs_t* musb = MUSB_REGS(rhport);

#if MUSB_CFG_DYNAMIC_FIFO
  alloced_fifo_bytes = CFG_TUD_ENDPOINT0_SIZE;
#endif

  /* When bmRequestType is REQUEST_TYPE_INVALID(0xFF), a control transfer state is SETUP or STATUS stage. */
  _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
  _dcd.status_out = 0;
  /* When EP0 pipe buf has not NULL, DATA stage works in progress. */
  pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);
  pipe0->buf = NULL;

  musb->intr_txen = 1; /* Enable only EP0 */
  musb->intr_rxen = 0;

  /* Clear FIFO settings */
  for (unsigned i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    musb->index = i;
    hwfifo_reset(musb, i, 0);
    hwfifo_reset(musb, i, 1);
  }
  dcd_event_bus_reset(rhport, (musb->power & MUSB_POWER_HSMODE) ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL, true);
}

/*------------------------------------------------------------------
 * Device API
 *------------------------------------------------------------------*/

#if CFG_TUSB_DEBUG >= MUSB_DEBUG
static void print_musb_info(musb_regs_t* musb_regs) {
  // print version, epinfo, raminfo, config_data0, fifo_size
  TU_LOG1("musb version = %u.%u\r\n", musb_regs->hwvers_bit.major, musb_regs->hwvers_bit.minor);
  TU_LOG1("Number of endpoints: %u TX, %u RX\r\n", musb_regs->epinfo_bit.tx_ep_num, musb_regs->epinfo_bit.rx_ep_num);
  TU_LOG1("RAM Info: %u DMA Channel, %u RAM address width\r\n", musb_regs->raminfo_bit.dma_channel, musb_regs->raminfo_bit.ram_bits);

  musb_regs->index = 0;
  TU_LOG1("config_data0 = 0x%x\r\n", musb_regs->indexed_csr.config_data0);

#if MUSB_CFG_DYNAMIC_FIFO
  TU_LOG1("Dynamic FIFO configuration\r\n");
#else
  for (uint8_t i=1; i <= musb_regs->epinfo_bit.tx_ep_num; i++) {
    musb_regs->index = i;
    TU_LOG1("FIFO %u Size: TX %u RX %u\r\n", i, musb_regs->indexed_csr.fifo_size_bit.tx, musb_regs->indexed_csr.fifo_size_bit.rx);
  }
#endif
}
#endif

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  musb_regs_t* musb_regs = MUSB_REGS(rhport);

#if CFG_TUSB_DEBUG >= MUSB_DEBUG
  print_musb_info(musb_regs);
#endif

  musb_regs->intr_usben |= MUSB_IE_SUSPND;
  musb_dcd_int_clear(rhport);
  musb_dcd_phy_init(rhport);
  dcd_connect(rhport);
  return true;
}

void dcd_int_enable(uint8_t rhport) {
  musb_dcd_int_enable(rhport);
}

void dcd_int_disable(uint8_t rhport) {
  musb_dcd_int_disable(rhport);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void)dev_addr;
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, 0);
  pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);

  pipe0->buf       = NULL;
  pipe0->length    = 0;
  pipe0->remaining = 0;
  /* Clear RX FIFO to return ACK. */
  ep_csr->csr0l = MUSB_CSRL0_RXRDYC | MUSB_CSRL0_DATAEND;
}

// Wake up host
void dcd_remote_wakeup(uint8_t rhport) {
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_regs->power |= MUSB_POWER_RESUME;

  unsigned cnt = SystemCoreClock / 1000;
  while (cnt--) __NOP();

  musb_regs->power &= ~MUSB_POWER_RESUME;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_regs->power |= TUD_OPT_HIGH_SPEED ? MUSB_POWER_HSENAB : 0;
  musb_regs->power |= MUSB_POWER_SOFTCONN;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_regs->power &= ~MUSB_POWER_SOFTCONN;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned epdir  = tu_edpt_dir(ep_addr);
  const unsigned mps     = tu_edpt_packet_size(ep_desc);

  pipe_state_t *pipe = pipe_get(epn, epdir);
  pipe->buf       = NULL;
  pipe->length    = 0;
  pipe->remaining = 0;
  pipe->armed     = false;

  musb_regs_t* musb = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb, epn);
  const uint8_t is_rx = (1 - epdir);
  musb_ep_maxp_csr_t* maxp_csr = &ep_csr->maxp_csr[is_rx];

  maxp_csr->maxp = mps;
  maxp_csr->csrh = 0;
#if MUSB_CFG_SHARED_FIFO
  if (epdir) {
    maxp_csr->csrh |= MUSB_CSRH_TX_MODE;
  }
#endif

  hwfifo_flush(musb, epn, is_rx, true);

  TU_ASSERT(hwfifo_config(musb, epn, is_rx, mps, ep_desc->bmAttributes.xfer == TUSB_XFER_BULK));
  musb->intren_ep[is_rx] |= TU_BIT(epn);

  return true;
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  const unsigned epn    = tu_edpt_number(ep_addr);
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  musb_regs_t* musb = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb, epn);
  const uint8_t is_rx = 1 - dir_in;
  ep_csr->maxp_csr[is_rx].csrh = 0;
  return hwfifo_config(musb, epn, is_rx, largest_packet_size, true);
}

bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc ) {
  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned dir_in  = tu_edpt_dir(ep_addr);
  const unsigned mps     = tu_edpt_packet_size(ep_desc);

  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);

  pipe_state_t *pipe = pipe_get(epn, dir_in);
  pipe->buf       = NULL;
  pipe->length    = 0;
  pipe->remaining = 0;
  pipe->armed     = false;

  musb_regs_t* musb = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb, epn);
  const uint8_t is_rx = 1 - dir_in;
  musb_ep_maxp_csr_t* maxp_csr = &ep_csr->maxp_csr[is_rx];

  maxp_csr->maxp = mps;
  maxp_csr->csrh |= MUSB_CSRH_ISO;
#if MUSB_CFG_SHARED_FIFO
  if (dir_in) {
    maxp_csr->csrh |= MUSB_CSRH_TX_MODE;
  }
#endif

  hwfifo_flush(musb, epn, is_rx, true);

#if MUSB_CFG_DYNAMIC_FIFO
  // fifo space is already allocated, keep the address and just change packet size
  musb->fifo_size[is_rx] = hwfifo_byte2size(mps) | MUSB_FIFOSZ_DOUBLE_PACKET;
#endif

  musb->intren_ep[is_rx] |= TU_BIT(epn);

  if (ie) musb_dcd_int_enable(rhport);

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  musb_regs_t* musb = MUSB_REGS(rhport);
  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);

  musb->intr_txen = 1; /* Enable only EP0 */
  musb->intr_rxen = 0;
  for (unsigned i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    musb_ep_csr_t* ep_csr = get_ep_csr(musb, i);
    for (unsigned d = 0; d < 2; d++) {
      musb_ep_maxp_csr_t* maxp_csr = &ep_csr->maxp_csr[d];
      hwfifo_flush(musb, i, d, true);
      hwfifo_reset(musb, i, d);
      maxp_csr->maxp = 0;
      maxp_csr->csrh = 0;
    }
  }

#if MUSB_CFG_DYNAMIC_FIFO
  alloced_fifo_bytes = CFG_TUD_ENDPOINT0_SIZE;
#endif

  if (ie) musb_dcd_int_enable(rhport);
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes, bool is_isr)
{
  (void)rhport;
  bool ret;
  unsigned const epnum = tu_edpt_number(ep_addr);
  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);

  if (epnum) {
    ret = edpt_n_xfer(rhport, ep_addr, buffer, total_bytes, false, is_isr);
  } else {
    ret = edpt0_xfer(rhport, ep_addr, buffer, total_bytes, is_isr);
  }

  if (ie) {
    musb_dcd_int_enable(rhport);
  }
  return ret;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack
// - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes, bool is_isr)
{
  (void)rhport;
  bool ret;
  unsigned const epnum = tu_edpt_number(ep_addr);
  TU_ASSERT(epnum);
  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);
  ret = edpt_n_xfer(rhport, ep_addr, ff, total_bytes, true, is_isr);
  if (ie) musb_dcd_int_enable(rhport);
  return ret;
}

// Stall endpoint
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);

  unsigned const epn = tu_edpt_number(ep_addr);
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, epn);

  if (0 == epn) {
    if (!ep_addr) { /* Ignore EP80 */
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      pipe_state_t* pipe0 = pipe_get(0, TUSB_DIR_OUT);
      pipe0->buf = NULL;
      ep_csr->csr0l = MUSB_CSRL0_STALL;
    }
  } else {
    const uint8_t is_rx = 1 - tu_edpt_dir(ep_addr);
    ep_csr->maxp_csr[is_rx].csrl = MUSB_CSRL_SEND_STALL(is_rx);
    pipe_state_t* pipe = pipe_get(epn, tu_edpt_dir(ep_addr));
    pipe->armed = false;
  }

  if (ie) musb_dcd_int_enable(rhport);
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const ie = musb_dcd_get_int_enable(rhport);
  musb_dcd_int_disable(rhport);

  unsigned const epn = tu_edpt_number(ep_addr);
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  musb_ep_csr_t* ep_csr = get_ep_csr(musb_regs, epn);
  const uint8_t is_rx = 1 - tu_edpt_dir(ep_addr);

  ep_csr->maxp_csr[is_rx].csrl = MUSB_CSRL_CLEAR_DATA_TOGGLE(is_rx);

  if (ie) musb_dcd_int_enable(rhport);
}

/*-------------------------------------------------------------------
 * ISR
 *-------------------------------------------------------------------*/
void dcd_int_handler(uint8_t rhport) {
  musb_regs_t* musb_regs = MUSB_REGS(rhport);
  const uint8_t saved_index = musb_regs->index; // save endpoint index

  //Part specific ISR setup/entry
  musb_dcd_int_handler_enter(rhport);

  uint_fast8_t intr_usb = musb_regs->intr_usb; // a read will clear this interrupt status
  uint_fast8_t intr_tx = musb_regs->intr_tx; // a read will clear this interrupt status
  uint_fast8_t intr_rx = musb_regs->intr_rx; // a read will clear this interrupt status
  // TU_LOG1("D%2x T%2x R%2x\r\n", is, txis, rxis);

  intr_usb &= musb_regs->intr_usben; /* Clear disabled interrupts */
  if (intr_usb & MUSB_IS_DISCON) {
  }
  if (intr_usb & MUSB_IS_SOF) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }
  if (intr_usb & MUSB_IS_RESET) {
    process_bus_reset(rhport);
  }
  if (intr_usb & MUSB_IS_RESUME) {
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }
  if (intr_usb & MUSB_IS_SUSPEND) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }

  intr_tx &= musb_regs->intr_txen; /* Clear disabled interrupts */
  if (intr_tx & TU_BIT(0)) {
    process_ep0(rhport);
    intr_tx &= ~TU_BIT(0);
  }

  while (intr_tx) {
    const unsigned epnum = __builtin_ctz(intr_tx);
    process_epin(rhport, musb_regs, epnum);
    intr_tx &= ~TU_BIT(epnum);

    // Double packet endpoint: TxPktRdy is clear, and interrupt is generated immediately when 1st packet is written.
    uint_fast8_t new_intr_tx = musb_regs->intr_tx;
    new_intr_tx &= musb_regs->intr_txen;

    intr_tx |= new_intr_tx;
  }

  intr_rx &= musb_regs->intr_rxen; /* Clear disabled interrupts */
  while (intr_rx) {
    unsigned const epnum = __builtin_ctz(intr_rx);
    process_epout(rhport, musb_regs, epnum, true);
    intr_rx &= ~TU_BIT(epnum);

    // Double packet endpoint: RxPktRdy is set and interrupt is generated immediately if 2nd packet is received
    uint_fast8_t new_intr_rx = musb_regs->intr_rx;
    new_intr_rx &= musb_regs->intr_rxen;

    intr_rx |= new_intr_rx;
  }

  musb_regs->index = saved_index; // restore endpoint index
}

#endif
