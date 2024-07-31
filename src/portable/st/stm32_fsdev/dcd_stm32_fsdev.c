/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
 *
 * Portions:
 * Copyright (c) 2016 STMicroelectronics
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2022 Simon Küppers (skuep)
 * Copyright (c) 2022 HiFiPhile
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

/**********************************************
 * This driver has been tested with the following MCUs:
 *  - F070, F072, L053, F042F6
 *
 * It also should work with minimal changes for any ST MCU with an "USB A"/"PCD"/"HCD" peripheral. This
 *  covers:
 *
 * F04x, F072, F078, F070x6/B     1024 byte buffer
 * F102, F103                      512 byte buffer; no internal D+ pull-up (maybe many more changes?)
 * F302xB/C, F303xB/C, F373        512 byte buffer; no internal D+ pull-up
 * F302x6/8, F302xD/E2, F303xD/E  1024 byte buffer; no internal D+ pull-up
 * G0                             2048 byte buffer; 32-bit bus; host mode
 * G4                             1024 byte buffer
 * H5                             2048 byte buffer; 32-bit bus; host mode
 * L0x2, L0x3                     1024 byte buffer
 * L1                              512 byte buffer
 * L4x2, L4x3                     1024 byte buffer
 * L5                             1024 byte buffer
 * U0                             1024 byte buffer; 32-bit bus
 * U535, U545                     2048 byte buffer; 32-bit bus; host mode
 * WB35, WB55                     1024 byte buffer
 *
 * To use this driver, you must:
 * - If you are using a device with crystal-less USB, set up the clock recovery system (CRS)
 * - Remap pins to be D+/D- on devices that they are shared (for example: F042Fx)
 *   - This is different to the normal "alternate function" GPIO interface, needs to go through SYSCFG->CFGRx register
 * - Enable USB clock; Perhaps use __HAL_RCC_USB_CLK_ENABLE();
 * - (Optionally configure GPIO HAL to tell it the USB driver is using the USB pins)
 * - call tusb_init();
 * - periodically call tusb_task();
 *
 * Assumptions of the driver:
 * - You are not using CAN (it must share the packet buffer)
 * - APB clock is >= 10 MHz
 * - On some boards, series resistors are required, but not on others.
 * - On some boards, D+ pull up resistor (1.5kohm) is required, but not on others.
 * - You don't have long-running interrupts; some USB packets must be quickly responded to.
 * - You have the ST CMSIS library linked into the project. HAL is not used.
 *
 * Current driver limitations (i.e., a list of features for you to add):
 * - STALL handled, but not tested.
 *   - Does it work? No clue.
 * - All EP BTABLE buffers are created based on max packet size of first EP opened with that address.
 * - Packet buffer memory is copied in the interrupt.
 *   - This is better for performance, but means interrupts are disabled for longer
 *   - DMA may be the best choice, but it could also be pushed to the USBD task.
 * - No double-buffering
 * - No DMA
 * - Minimal error handling
 *   - Perhaps error interrupts should be reported to the stack, or cause a device reset?
 * - Assumes a single USB peripheral; I think that no hardware has multiple so this is fine.
 * - Add a callback for enabling/disabling the D+ PU on devices without an internal PU.
 * - F3 models use three separate interrupts. I think we could only use the LP interrupt for
 *     everything?  However, the interrupts are configurable so the DisableInt and EnableInt
 *     below functions could be adjusting the wrong interrupts (if they had been reconfigured)
 * - LPM is not used correctly, or at all?
 *
 * USB documentation and Reference implementations
 * - STM32 Reference manuals
 * - STM32 USB Hardware Guidelines AN4879
 *
 * - STM32 HAL (much of this driver is based on this)
 * - libopencm3/lib/stm32/common/st_usbfs_core.c
 * - Keil USB Device http://www.keil.com/pack/doc/mw/USB/html/group__usbd.html
 *
 * - YouTube OpenTechLab 011; https://www.youtube.com/watch?v=4FOkJLp_PUw
 *
 * Advantages over HAL driver:
 * - Tiny (saves RAM, assumes a single USB peripheral)
 *
 * Notes:
 * - The buffer table is allocated as endpoints are opened. The allocation is only
 *   cleared when the device is reset. This may be bad if the USB device needs
 *   to be reconfigured.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && defined(TUP_USBIP_FSDEV) && \
    !(defined(TUP_USBIP_FSDEV_CH32) && CFG_TUD_WCH_USBIP_FSDEV == 0)

#include "device/dcd.h"

#if defined(TUP_USBIP_FSDEV_STM32)
  // Undefine to reduce the dependence on HAL
  #undef USE_HAL_DRIVER
  #include "fsdev_stm32.h"
#elif defined(TUP_USBIP_FSDEV_CH32)
  #include "fsdev_ch32.h"
#else
  #error "Unknown USB IP"
#endif

#include "fsdev_common.h"

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+



//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// One of these for every EP IN & OUT, uses a bit of RAM....
typedef struct {
  uint8_t *buffer;
  tu_fifo_t *ff;
  uint16_t total_len;
  uint16_t queued_len;
  uint16_t max_packet_size;
  uint8_t ep_idx;   // index for USB_EPnR register
  bool iso_in_sending; // Workaround for ISO IN EP doesn't have interrupt mask
} xfer_ctl_t;

// EP allocator
typedef struct {
  uint8_t ep_num;
  uint8_t ep_type;
  bool allocated[2];
} ep_alloc_t;

static xfer_ctl_t xfer_status[CFG_TUD_ENDPPOINT_MAX][2];
static ep_alloc_t ep_alloc_status[FSDEV_EP_COUNT];

static uint8_t remoteWakeCountdown; // When wake is requested

//--------------------------------------------------------------------+
// Prototypes
//--------------------------------------------------------------------+

// into the stack.
static void handle_bus_reset(uint8_t rhport);
static void dcd_transmit_packet(xfer_ctl_t *xfer, uint16_t ep_ix);
static bool edpt_xfer(uint8_t rhport, uint8_t ep_addr);
static void dcd_ep_ctr_handler(void);

// PMA allocation/access
static uint16_t ep_buf_ptr; ///< Points to first free memory location
static uint32_t dcd_pma_alloc(uint16_t len, bool dbuf);
static uint8_t dcd_ep_alloc(uint8_t ep_addr, uint8_t ep_type);
static bool dcd_write_packet_memory(uint16_t dst, const void *__restrict src, uint16_t wNBytes);
static bool dcd_read_packet_memory(void *__restrict dst, uint16_t src, uint16_t wNBytes);

static bool dcd_write_packet_memory_ff(tu_fifo_t *ff, uint16_t dst, uint16_t wNBytes);
static bool dcd_read_packet_memory_ff(tu_fifo_t *ff, uint16_t src, uint16_t wNBytes);

TU_ATTR_UNUSED static void edpt0_open(uint8_t rhport);

//--------------------------------------------------------------------+
// Inline helper
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline xfer_ctl_t *xfer_ctl_ptr(uint32_t ep_addr)
{
  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);
  // Fix -Werror=null-dereference
  TU_ASSERT(epnum < CFG_TUD_ENDPPOINT_MAX, &xfer_status[0][0]);

  return &xfer_status[epnum][dir];
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+
void dcd_init(uint8_t rhport) {
  // Follow the RM mentions to use a special ordering of PDWN and FRES
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  // Perform USB peripheral reset
  USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait startup time, for F042 and F070, this is <= 1 us.
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }
  USB->CNTR = 0; // Enable USB

#if !defined(STM32G0) && !defined(STM32H5) && !defined(STM32U5)
  // BTABLE register does not exist any more on STM32G0, it is fixed to USB SRAM base address
  USB->BTABLE = FSDEV_BTABLE_BASE;
#endif

  USB->ISTR = 0; // Clear pending interrupts

  // Reset endpoints to disabled
  for (uint32_t i = 0; i < FSDEV_EP_COUNT; i++) {
    // This doesn't clear all bits since some bits are "toggle", but does set the type to DISABLED.
    pcd_set_endpoint(USB, i, 0u);
  }

  USB->CNTR |= USB_CNTR_RESETM | USB_CNTR_ESOFM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
  handle_bus_reset(rhport);

  // Enable pull-up if supported
  dcd_connect(rhport);
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void)rhport;

  if (en) {
    USB->CNTR |= USB_CNTR_SOFM;
  } else {
    USB->CNTR &= ~USB_CNTR_SOFM;
  }
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void)dev_addr;

  // Respond with status
  dcd_edpt_xfer(rhport, TUSB_DIR_IN_MASK | 0x00, NULL, 0);

  // DCD can only set address after status for this request is complete.
  // do it at dcd_edpt0_status_complete()
}

void dcd_remote_wakeup(uint8_t rhport) {
  (void)rhport;

  USB->CNTR |= USB_CNTR_RESUME;
  remoteWakeCountdown = 4u; // required to be 1 to 15 ms, ESOF should trigger every 1ms.
}

static void handle_bus_reset(uint8_t rhport) {
  USB->DADDR = 0u; // disable USB Function

  for (uint32_t i = 0; i < FSDEV_EP_COUNT; i++) {
    // Clear EP allocation status
    ep_alloc_status[i].ep_num = 0xFF;
    ep_alloc_status[i].ep_type = 0xFF;
    ep_alloc_status[i].allocated[0] = false;
    ep_alloc_status[i].allocated[1] = false;
  }

  // Reset PMA allocation
  ep_buf_ptr = FSDEV_BTABLE_BASE + 8 * FSDEV_EP_COUNT;

  edpt0_open(rhport); // open control endpoint (both IN & OUT)

  USB->DADDR = USB_DADDR_EF; // Enable USB Function
}

// Handle CTR interrupt for the TX/IN direction
// Upon call, (wIstr & USB_ISTR_DIR) == 0U
static void dcd_ep_ctr_tx_handler(uint32_t wIstr) {
  uint32_t EPindex = wIstr & USB_ISTR_EP_ID;
  uint32_t wEPRegVal = pcd_get_endpoint(USB, EPindex);
  uint8_t ep_addr = (wEPRegVal & USB_EPADDR_FIELD) | TUSB_DIR_IN_MASK;

  // Verify the CTR_TX bit is set. This was in the ST Micro code,
  // but I'm not sure it's actually necessary?
  if ((wEPRegVal & USB_EP_CTR_TX) == 0U) {
    return;
  }

  /* clear int flag */
  pcd_clear_tx_ep_ctr(USB, EPindex);

  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);

  if ((wEPRegVal & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS) {
    // Ignore spurious interrupts that we don't schedule
    // host can send IN token while there is no data to send, since ISO does not have NAK
    // this will result to zero length packet --> trigger interrupt (which cannot be masked)
    if (!xfer->iso_in_sending) {
      return;
    }
    xfer->iso_in_sending = false;
    uint8_t buf_id = (wEPRegVal & USB_EP_DTOG_TX) ? 0 : 1;
    btable_set_count(EPindex, buf_id, 0);
  }

  if ((xfer->total_len != xfer->queued_len)) {
    dcd_transmit_packet(xfer, EPindex);
  } else {
    dcd_event_xfer_complete(0, ep_addr, xfer->total_len, XFER_RESULT_SUCCESS, true);
  }
}

// Handle CTR interrupt for the RX/OUT direction
// Upon call, (wIstr & USB_ISTR_DIR) == 0U
static void dcd_ep_ctr_rx_handler(uint32_t wIstr)
{
#ifdef FSDEV_BUS_32BIT
  /* https://www.st.com/resource/en/errata_sheet/es0561-stm32h503cbebkbrb-device-errata-stmicroelectronics.pdf
   * From STM32H503 errata 2.15.1: Buffer description table update completes after CTR interrupt triggers
   * Description:
   * - During OUT transfers, the correct transfer interrupt (CTR) is triggered a little before the last USB SRAM accesses
   * have completed. If the software responds quickly to the interrupt, the full buffer contents may not be correct.
   * Workaround:
   * - Software should ensure that a small delay is included before accessing the SRAM contents. This delay
   * should be 800 ns in Full Speed mode and 6.4 μs in Low Speed mode
   * - Since H5 can run up to 250Mhz -> 1 cycle = 4ns. Per errata, we need to wait 200 cycles. Though executing code
   * also takes time, so we'll wait 60 cycles (count = 20).
   * - Since Low Speed mode is not supported/popular, we will ignore it for now.
   *
   * Note: this errata also seems to apply to G0, U5, H5 etc.
   */
  volatile uint32_t cycle_count = 20; // defined as PCD_RX_PMA_CNT in stm32 hal_driver
  while (cycle_count > 0U) {
    cycle_count--; // each count take 3 cycles (1 for sub, jump, and compare)
  }
#endif

  uint32_t EPindex = wIstr & USB_ISTR_EP_ID;
  uint32_t wEPRegVal = pcd_get_endpoint(USB, EPindex);
  uint8_t ep_addr = wEPRegVal & USB_EPADDR_FIELD;

  // Verify the CTR_RX bit is set. This was in the ST Micro code,
  // but I'm not sure it's actually necessary?
  if ((wEPRegVal & USB_EP_CTR_RX) == 0U) {
    return;
  }

  if (wEPRegVal & USB_EP_SETUP) {
    uint32_t count = btable_get_count(EPindex, BTABLE_BUF_RX);
    // Setup packet should always be 8 bytes. If not, ignore it, and try again.
    if (count == 8) {
      uint16_t rx_addr = btable_get_addr(EPindex, BTABLE_BUF_RX);
#ifdef FSDEV_BUS_32BIT
      dcd_event_setup_received(0, (uint8_t *)(USB_PMAADDR + rx_addr), true);
#else
      // The setup_received function uses memcpy, so this must first copy the setup data into
      // user memory, to allow for the 32-bit access that memcpy performs.
      uint32_t userMemBuf[2];
      dcd_read_packet_memory(userMemBuf, rx_addr, 8);
      dcd_event_setup_received(0, (uint8_t*) userMemBuf, true);
#endif

      // Reset EP to NAK (in case it had been stalling)
      wEPRegVal = ep_add_tx_status(wEPRegVal, USB_EP_TX_NAK);
      wEPRegVal = ep_add_rx_status(wEPRegVal, USB_EP_RX_NAK);
      wEPRegVal = ep_add_tx_dtog(wEPRegVal, 1);
      wEPRegVal = ep_add_rx_dtog(wEPRegVal, 1);
      pcd_set_endpoint(USB, 0, wEPRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX);
    }
  } else {
    // Clear RX CTR interrupt flag
    if (ep_addr != 0u) {
      pcd_clear_rx_ep_ctr(USB, EPindex);
    }

    xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);

    uint8_t buf_id;
    if ((wEPRegVal & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS) {
      // ISO endpoints are double buffered
      buf_id = (wEPRegVal & USB_EP_DTOG_RX) ? 0 : 1;
    } else {
      buf_id = 1;
    }
    uint32_t count = btable_get_count(EPindex, buf_id);
    uint16_t addr = (uint16_t) btable_get_addr(EPindex, buf_id);

    if (count != 0U) {
      if (xfer->ff) {
        dcd_read_packet_memory_ff(xfer->ff, addr, count);
      } else {
        dcd_read_packet_memory(xfer->buffer + xfer->queued_len, addr, count);
      }

      xfer->queued_len = (uint16_t)(xfer->queued_len + count);
    }

    if ((count < xfer->max_packet_size) || (xfer->queued_len == xfer->total_len)) {
      // all bytes received or short packet
      dcd_event_xfer_complete(0, ep_addr, xfer->queued_len, XFER_RESULT_SUCCESS, true);
    } else {
      /* Set endpoint active again for receiving more data.
       * Note that isochronous endpoints stay active always */
      if ((wEPRegVal & USB_EP_TYPE_MASK) != USB_EP_ISOCHRONOUS) {
        uint16_t remaining = xfer->total_len - xfer->queued_len;
        uint16_t cnt = tu_min16(remaining, xfer->max_packet_size);
        btable_set_rx_bufsize(EPindex, BTABLE_BUF_RX, cnt);
      }
      pcd_set_ep_rx_status(USB, EPindex, USB_EP_RX_VALID);
    }
  }

  // For EP0, prepare to receive another SETUP packet.
  // Clear CTR last so that a new packet does not overwrite the packing being read.
  // (Based on the docs, it seems SETUP will always be accepted after CTR is cleared)
  if (ep_addr == 0u) {
    // Always be prepared for a status packet...
    btable_set_rx_bufsize(EPindex, BTABLE_BUF_RX, CFG_TUD_ENDPOINT0_SIZE);
    pcd_clear_rx_ep_ctr(USB, EPindex);
  }
}

static void dcd_ep_ctr_handler(void)
{
  uint32_t wIstr;

  /* stay in loop while pending interrupts */
  while (((wIstr = USB->ISTR) & USB_ISTR_CTR) != 0U) {
    if ((wIstr & USB_ISTR_DIR) == 0U) {
      /* TX/IN */
      dcd_ep_ctr_tx_handler(wIstr);
    } else {
      /* RX/OUT*/
      dcd_ep_ctr_rx_handler(wIstr);
    }
  }
}

void dcd_int_handler(uint8_t rhport) {
  uint32_t int_status = USB->ISTR;
  // const uint32_t handled_ints = USB_ISTR_CTR | USB_ISTR_RESET | USB_ISTR_WKUP
  //     | USB_ISTR_SUSP | USB_ISTR_SOF | USB_ISTR_ESOF;
  //  unused IRQs: (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_L1REQ )

  // The ST driver loops here on the CTR bit, but that loop has been moved into the
  // dcd_ep_ctr_handler(), so less need to loop here. The other interrupts shouldn't
  // be triggered repeatedly.

  /* Put SOF flag at the beginning of ISR in case to get least amount of jitter if it is used for timing purposes */
  if (int_status & USB_ISTR_SOF) {
    USB->ISTR = (fsdev_bus_t)~USB_ISTR_SOF;
    dcd_event_sof(0, USB->FNR & USB_FNR_FN, true);
  }

  if (int_status & USB_ISTR_RESET) {
    // USBRST is start of reset.
    USB->ISTR = (fsdev_bus_t)~USB_ISTR_RESET;
    handle_bus_reset(rhport);
    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
    return; // Don't do the rest of the things here; perhaps they've been cleared?
  }

  if (int_status & USB_ISTR_CTR) {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    dcd_ep_ctr_handler();
  }

  if (int_status & USB_ISTR_WKUP) {
    USB->CNTR &= ~USB_CNTR_LPMODE;
    USB->CNTR &= ~USB_CNTR_FSUSP;

    USB->ISTR = (fsdev_bus_t)~USB_ISTR_WKUP;
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
  }

  if (int_status & USB_ISTR_SUSP) {
    /* Suspend is asserted for both suspend and unplug events. without Vbus monitoring,
     * these events cannot be differentiated, so we only trigger suspend. */

    /* Force low-power mode in the macrocell */
    USB->CNTR |= USB_CNTR_FSUSP;
    USB->CNTR |= USB_CNTR_LPMODE;

    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    USB->ISTR = (fsdev_bus_t)~USB_ISTR_SUSP;
    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
  }

  if (int_status & USB_ISTR_ESOF) {
    if (remoteWakeCountdown == 1u) {
      USB->CNTR &= ~USB_CNTR_RESUME;
    }
    if (remoteWakeCountdown > 0u) {
      remoteWakeCountdown--;
    }
    USB->ISTR = (fsdev_bus_t)~USB_ISTR_ESOF;
  }
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request) {
  (void)rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
      request->bRequest == TUSB_REQ_SET_ADDRESS) {
    uint8_t const dev_addr = (uint8_t)request->wValue;
    USB->DADDR = (USB_DADDR_EF | dev_addr);
  }
}

/***
 * Allocate a section of PMA
 * In case of double buffering, high 16bit is the address of 2nd buffer
 * During failure, TU_ASSERT is used. If this happens, rework/reallocate memory manually.
 */
static uint32_t dcd_pma_alloc(uint16_t len, bool dbuf)
{
  uint8_t blsize, num_block;
  uint16_t aligned_len = pma_align_buffer_size(len, &blsize, &num_block);
  (void) blsize;
  (void) num_block;

  uint32_t addr = ep_buf_ptr;
  ep_buf_ptr = (uint16_t)(ep_buf_ptr + aligned_len); // increment buffer pointer

  if (dbuf) {
    addr |= ((uint32_t)ep_buf_ptr) << 16;
    ep_buf_ptr = (uint16_t)(ep_buf_ptr + aligned_len); // increment buffer pointer
  }

  // Verify packet buffer is not overflowed
  TU_ASSERT(ep_buf_ptr <= FSDEV_PMA_SIZE, 0xFFFF);

  return addr;
}

/***
 * Allocate hardware endpoint
 */
static uint8_t dcd_ep_alloc(uint8_t ep_addr, uint8_t ep_type)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  for (uint8_t i = 0; i < FSDEV_EP_COUNT; i++) {
    // Check if already allocated
    if (ep_alloc_status[i].allocated[dir] &&
        ep_alloc_status[i].ep_type == ep_type &&
        ep_alloc_status[i].ep_num == epnum) {
      return i;
    }

    // If EP of current direction is not allocated
    // Except for ISO endpoint, both direction should be free
    if (!ep_alloc_status[i].allocated[dir] &&
        (ep_type != TUSB_XFER_ISOCHRONOUS || !ep_alloc_status[i].allocated[dir ^ 1])) {
      // Check if EP number is the same
      if (ep_alloc_status[i].ep_num == 0xFF || ep_alloc_status[i].ep_num == epnum) {
        // One EP pair has to be the same type
        if (ep_alloc_status[i].ep_type == 0xFF || ep_alloc_status[i].ep_type == ep_type) {
          ep_alloc_status[i].ep_num = epnum;
          ep_alloc_status[i].ep_type = ep_type;
          ep_alloc_status[i].allocated[dir] = true;

          return i;
        }
      }
    }
  }

  // Allocation failed
  TU_ASSERT(0);
}

void edpt0_open(uint8_t rhport) {
  (void) rhport;

  dcd_ep_alloc(0x0, TUSB_XFER_CONTROL);
  dcd_ep_alloc(0x80, TUSB_XFER_CONTROL);

  xfer_status[0][0].max_packet_size = CFG_TUD_ENDPOINT0_SIZE;
  xfer_status[0][0].ep_idx = 0;

  xfer_status[0][1].max_packet_size = CFG_TUD_ENDPOINT0_SIZE;
  xfer_status[0][1].ep_idx = 0;

  uint16_t pma_addr0 = dcd_pma_alloc(CFG_TUD_ENDPOINT0_SIZE, false);
  uint16_t pma_addr1 = dcd_pma_alloc(CFG_TUD_ENDPOINT0_SIZE, false);

  btable_set_addr(0, BTABLE_BUF_RX, pma_addr0);
  btable_set_addr(0, BTABLE_BUF_TX, pma_addr1);

  uint32_t ep_reg = FSDEV_REG->ep[0].reg & ~USB_EPREG_MASK;
  ep_reg |= USB_EP_CONTROL | USB_EP_CTR_RX | USB_EP_CTR_TX;
  ep_reg = ep_add_tx_status(ep_reg, USB_EP_TX_NAK);
  ep_reg = ep_add_rx_status(ep_reg, USB_EP_RX_NAK);
  // no need to explicitly set DTOG bits since we aren't masked DTOG bit

  pcd_set_endpoint(USB, 0, ep_reg);
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_ep) {
  (void)rhport;
  uint8_t const ep_addr = desc_ep->bEndpointAddress;
  uint8_t const dir = tu_edpt_dir(ep_addr);
  const uint16_t packet_size = tu_edpt_packet_size(desc_ep);
  uint8_t const ep_idx = dcd_ep_alloc(ep_addr, desc_ep->bmAttributes.xfer);
  TU_ASSERT(ep_idx < FSDEV_EP_COUNT);

  uint32_t ep_reg = FSDEV_REG->ep[ep_idx].reg & ~USB_EPREG_MASK;
  ep_reg |= tu_edpt_number(ep_addr) | USB_EP_CTR_RX | USB_EP_CTR_TX;

  // Set type
  switch (desc_ep->bmAttributes.xfer) {
    case TUSB_XFER_BULK:
      ep_reg |= USB_EP_CONTROL; // FIXME should it be bulk?
      break;
    case TUSB_XFER_INTERRUPT:
      ep_reg |= USB_EP_INTERRUPT;
      break;

    default:
      // Note: ISO endpoint should use alloc / active functions
      TU_ASSERT(false);
  }

  /* Create a packet memory buffer area. */
  uint16_t pma_addr = dcd_pma_alloc(packet_size, false);
  btable_set_addr(ep_idx, dir == TUSB_DIR_IN ? BTABLE_BUF_TX : BTABLE_BUF_RX, pma_addr);

  xfer_ctl_ptr(ep_addr)->max_packet_size = packet_size;
  xfer_ctl_ptr(ep_addr)->ep_idx = ep_idx;

  if (dir == TUSB_DIR_IN) {
    ep_reg = ep_add_tx_status(ep_reg, USB_EP_TX_NAK);
    ep_reg = ep_add_tx_dtog(ep_reg, 0);
    ep_reg &= ~(USB_EPRX_STAT | USB_EP_DTOG_RX);
  } else {
    ep_reg = ep_add_rx_status(ep_reg, USB_EP_RX_NAK);
    ep_reg = ep_add_rx_dtog(ep_reg, 0);
    ep_reg &= ~(USB_EPTX_STAT | USB_EP_DTOG_TX);
  }
  pcd_set_endpoint(USB, ep_idx, ep_reg);

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  (void)rhport;

  for (uint32_t i = 1; i < FSDEV_EP_COUNT; i++) {
    // Reset endpoint
    pcd_set_endpoint(USB, i, 0);
    // Clear EP allocation status
    ep_alloc_status[i].ep_num = 0xFF;
    ep_alloc_status[i].ep_type = 0xFF;
    ep_alloc_status[i].allocated[0] = false;
    ep_alloc_status[i].allocated[1] = false;
  }

  // Reset PMA allocation
  ep_buf_ptr = FSDEV_BTABLE_BASE + 8 * CFG_TUD_ENDPPOINT_MAX + 2 * CFG_TUD_ENDPOINT0_SIZE;
}

/**
 * Close an endpoint.
 *
 * This function may be called with interrupts enabled or disabled.
 *
 * This also clears transfers in progress, should there be any.
 */
void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;

  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);
  uint8_t const ep_idx = xfer->ep_idx;
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (dir == TUSB_DIR_IN) {
    pcd_set_ep_tx_status(USB, ep_idx, USB_EP_TX_DIS);
  } else {
    pcd_set_ep_rx_status(USB, ep_idx, USB_EP_RX_DIS);
  }
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  (void)rhport;

  uint8_t const ep_idx = dcd_ep_alloc(ep_addr, TUSB_XFER_ISOCHRONOUS);

  /* Create a packet memory buffer area. Enable double buffering for devices with 2048 bytes PMA,
     for smaller devices double buffering occupy too much space. */
  uint32_t pma_addr = dcd_pma_alloc(largest_packet_size, true);
#if FSDEV_PMA_SIZE > 1024u
  uint16_t pma_addr2 = pma_addr >> 16;
#else
  uint16_t pma_addr2 = pma_addr;
#endif

  btable_set_addr(ep_idx, 0, pma_addr);
  btable_set_addr(ep_idx, 1, pma_addr2);
  xfer_ctl_ptr(ep_addr)->ep_idx = ep_idx;

  pcd_set_eptype(USB, ep_idx, USB_EP_ISOCHRONOUS);

  return true;
}

bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const *desc_ep) {
  (void)rhport;
  uint8_t const ep_addr = desc_ep->bEndpointAddress;
  uint8_t const ep_idx = xfer_ctl_ptr(ep_addr)->ep_idx;
  uint8_t const dir = tu_edpt_dir(ep_addr);

  xfer_ctl_ptr(ep_addr)->max_packet_size = tu_edpt_packet_size(desc_ep);

  uint32_t ep_reg = FSDEV_REG->ep[0].reg & ~USB_EPREG_MASK;
  ep_reg |= tu_edpt_number(ep_addr) | USB_EP_ISOCHRONOUS | USB_EP_CTR_RX | USB_EP_CTR_TX;
  ep_reg = ep_add_tx_status(ep_reg, USB_EP_TX_DIS);
  ep_reg = ep_add_rx_status(ep_reg, USB_EP_RX_DIS);

  // no need to explicitly set DTOG bits since we aren't masked DTOG bit
  if (dir == TUSB_DIR_IN) {
    ep_reg = ep_add_rx_dtog(ep_reg, 1);
  } else {
    ep_reg = ep_add_tx_dtog(ep_reg, 1);
  }

  pcd_set_endpoint(USB, ep_idx, ep_reg);

  return true;
}

// Currently, single-buffered, and only 64 bytes at a time (max)
static void dcd_transmit_packet(xfer_ctl_t *xfer, uint16_t ep_ix) {
  uint16_t len = (uint16_t)(xfer->total_len - xfer->queued_len);
  if (len > xfer->max_packet_size) {
    len = xfer->max_packet_size;
  }

  uint16_t ep_reg = pcd_get_endpoint(USB, ep_ix);
  bool const is_iso = (ep_reg & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS;

  uint8_t buf_id;
  if (is_iso) {
    buf_id = (ep_reg & USB_EP_DTOG_TX) ? 1 : 0;
  } else {
    buf_id = BTABLE_BUF_TX;
  }
  uint16_t addr_ptr = (uint16_t) btable_get_addr(ep_ix, buf_id);
  btable_set_count(ep_ix, buf_id, len);

  if (xfer->ff) {
    dcd_write_packet_memory_ff(xfer->ff, addr_ptr, len);
  } else {
    dcd_write_packet_memory(addr_ptr, &(xfer->buffer[xfer->queued_len]), len);
  }
  xfer->queued_len = (uint16_t)(xfer->queued_len + len);

  dcd_int_disable(0);
  pcd_set_ep_tx_status(USB, ep_ix, USB_EP_TX_VALID);
  if (is_iso) {
    xfer->iso_in_sending = true;
  }
  dcd_int_enable(0);
}

static bool edpt_xfer(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;

  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);
  uint8_t const ep_idx = xfer->ep_idx;
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (dir == TUSB_DIR_IN) {
    dcd_transmit_packet(xfer, ep_idx);
  } else {
    uint32_t cnt = (uint32_t) tu_min16(xfer->total_len, xfer->max_packet_size);
    uint16_t ep_reg = pcd_get_endpoint(USB, ep_idx);

    if ((ep_reg & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS) {
      btable_set_rx_bufsize(ep_idx, 0, cnt);
      btable_set_rx_bufsize(ep_idx, 1, cnt);
    } else {
      btable_set_rx_bufsize(ep_idx, BTABLE_BUF_RX, cnt);
    }

    pcd_set_ep_rx_status(USB, ep_idx, USB_EP_RX_VALID);
  }

  return true;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);

  xfer->buffer = buffer;
  xfer->ff = NULL;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;

  return edpt_xfer(rhport, ep_addr);
}

bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes)
{
  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);
  xfer->buffer = NULL;
  xfer->ff = ff;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;

  return edpt_xfer(rhport, ep_addr);
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;

  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);
  uint8_t const ep_idx = xfer->ep_idx;
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (dir == TUSB_DIR_IN) {
    pcd_set_ep_tx_status(USB, ep_idx, USB_EP_TX_STALL);
  } else {
    pcd_set_ep_rx_status(USB, ep_idx, USB_EP_RX_STALL);
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;

  xfer_ctl_t *xfer = xfer_ctl_ptr(ep_addr);
  uint8_t const ep_idx = xfer->ep_idx;
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (dir == TUSB_DIR_IN) { // IN
    if (pcd_get_eptype(USB, ep_idx) != USB_EP_ISOCHRONOUS) {
      pcd_set_ep_tx_status(USB, ep_idx, USB_EP_TX_NAK);
    }

    /* Reset to DATA0 if clearing stall condition. */
    pcd_clear_tx_dtog(USB, ep_idx);
  } else { // OUT
    if (pcd_get_eptype(USB, ep_idx) != USB_EP_ISOCHRONOUS) {
      pcd_set_ep_rx_status(USB, ep_idx, USB_EP_RX_NAK);
    }
    /* Reset to DATA0 if clearing stall condition. */
    pcd_clear_rx_dtog(USB, ep_idx);
  }
}

#ifdef FSDEV_BUS_32BIT
static bool dcd_write_packet_memory(uint16_t dst, const void *__restrict src, uint16_t wNBytes) {
  const uint8_t *src8 = src;
  volatile uint32_t *dst32 = (volatile uint32_t *)(USB_PMAADDR + dst);

  for (uint32_t n = wNBytes / 4; n > 0; --n) {
    *dst32++ = tu_unaligned_read32(src8);
    src8 += 4;
  }

  uint16_t odd = wNBytes & 0x03;
  if (odd) {
    uint32_t wrVal = *src8;
    odd--;

    if (odd) {
      wrVal |= *++src8 << 8;
      odd--;

      if (odd) {
        wrVal |= *++src8 << 16;
      }
    }

    *dst32 = wrVal;
  }

  return true;
}
#else
// Packet buffer access can only be 8- or 16-bit.
/**
 * @brief Copy a buffer from user memory area to packet memory area (PMA).
 *        This uses un-aligned for user memory and 16-bit access for packet memory.
 * @param   dst, byte address in PMA; must be 16-bit aligned
 * @param   src pointer to user memory area.
 * @param   wPMABufAddr address into PMA.
 * @param   wNBytes no. of bytes to be copied.
 * @retval None
 */
static bool dcd_write_packet_memory(uint16_t dst, const void *__restrict src, uint16_t wNBytes) {
  uint32_t n = (uint32_t)wNBytes >> 1U;
  const uint8_t *src8 = src;
  volatile uint16_t *pdw16 = &pma[FSDEV_PMA_STRIDE * (dst >> 1)];

  while (n--) {
    *pdw16 = tu_unaligned_read16(src8);
    src8 += 2;
    pdw16 += FSDEV_PMA_STRIDE;
  }

  if (wNBytes & 0x01) {
    *pdw16 = (uint16_t) *src8;
  }

  return true;
}
#endif

/**
 * @brief Copy from FIFO to packet memory area (PMA).
 *        Uses byte-access of system memory and 16-bit access of packet memory
 * @param   wNBytes no. of bytes to be copied.
 * @retval None
 */
static bool dcd_write_packet_memory_ff(tu_fifo_t *ff, uint16_t dst, uint16_t wNBytes) {
  // Since we copy from a ring buffer FIFO, a wrap might occur making it necessary to conduct two copies
  tu_fifo_buffer_info_t info;
  tu_fifo_get_read_info(ff, &info);

  uint16_t cnt_lin = TU_MIN(wNBytes, info.len_lin);
  uint16_t cnt_wrap = TU_MIN(wNBytes - cnt_lin, info.len_wrap);

  // We want to read from the FIFO and write it into the PMA, if LIN part is ODD and has WRAPPED part,
  // last lin byte will be combined with wrapped part
  // To ensure PMA is always access aligned (dst aligned to 16 or 32 bit)
#ifdef FSDEV_BUS_32BIT
  if ((cnt_lin & 0x03) && cnt_wrap) {
    // Copy first linear part
    dcd_write_packet_memory(dst, info.ptr_lin, cnt_lin & ~0x03);
    dst += cnt_lin & ~0x03;

    // Copy last linear bytes & first wrapped bytes to buffer
    uint32_t i;
    uint8_t tmp[4];
    for (i = 0; i < (cnt_lin & 0x03); i++) {
      tmp[i] = ((uint8_t *)info.ptr_lin)[(cnt_lin & ~0x03) + i];
    }
    uint32_t wCnt = cnt_wrap;
    for (; i < 4 && wCnt > 0; i++, wCnt--) {
      tmp[i] = *(uint8_t *)info.ptr_wrap;
      info.ptr_wrap = (uint8_t *)info.ptr_wrap + 1;
    }

    // Write unaligned buffer
    dcd_write_packet_memory(dst, &tmp, 4);
    dst += 4;

    // Copy rest of wrapped byte
    if (wCnt) {
      dcd_write_packet_memory(dst, info.ptr_wrap, wCnt);
    }
  }
#else
  if ((cnt_lin & 0x01) && cnt_wrap) {
    // Copy first linear part
    dcd_write_packet_memory(dst, info.ptr_lin, cnt_lin & ~0x01);
    dst += cnt_lin & ~0x01;

    // Copy last linear byte & first wrapped byte
    uint16_t tmp = ((uint8_t *)info.ptr_lin)[cnt_lin - 1] | ((uint16_t)(((uint8_t *)info.ptr_wrap)[0]) << 8U);
    dcd_write_packet_memory(dst, &tmp, 2);
    dst += 2;

    // Copy rest of wrapped byte
    dcd_write_packet_memory(dst, ((uint8_t *)info.ptr_wrap) + 1, cnt_wrap - 1);
  }
#endif
  else {
    // Copy linear part
    dcd_write_packet_memory(dst, info.ptr_lin, cnt_lin);
    dst += info.len_lin;

    if (info.len_wrap) {
      // Copy wrapped byte
      dcd_write_packet_memory(dst, info.ptr_wrap, cnt_wrap);
    }
  }

  tu_fifo_advance_read_pointer(ff, cnt_lin + cnt_wrap);

  return true;
}

#ifdef FSDEV_BUS_32BIT
static bool dcd_read_packet_memory(void *__restrict dst, uint16_t src, uint16_t wNBytes) {
  uint8_t *dst8 = dst;
  volatile uint32_t *src32 = (volatile uint32_t *)(USB_PMAADDR + src);

  for (uint32_t n = wNBytes / 4; n > 0; --n) {
    tu_unaligned_write32(dst8, *src32++);
    dst8 += 4;
  }

  uint16_t odd = wNBytes & 0x03;
  if (odd) {
    uint32_t rdVal = *src32;

    *dst8 = tu_u32_byte0(rdVal);
    odd--;

    if (odd) {
      *++dst8 = tu_u32_byte1(rdVal);
      odd--;

      if (odd) {
        *++dst8 = tu_u32_byte2(rdVal);
      }
    }
  }

  return true;
}
#else
/**
 * @brief Copy a buffer from packet memory area (PMA) to user memory area.
 *        Uses unaligned for system memory and 16-bit access of packet memory
 * @param   wNBytes no. of bytes to be copied.
 * @retval None
 */
static bool dcd_read_packet_memory(void *__restrict dst, uint16_t src, uint16_t wNBytes) {
  uint32_t n = (uint32_t)wNBytes >> 1U;
  volatile const uint16_t *pdw16 = &pma[FSDEV_PMA_STRIDE * (src >> 1)];
  uint8_t *dst8 = (uint8_t *)dst;

  while (n--) {
    tu_unaligned_write16(dst8, *pdw16);
    dst8 += 2;
    pdw16 += FSDEV_PMA_STRIDE;
  }

  if (wNBytes & 0x01) {
    *dst8++ = tu_u16_low(*pdw16);
  }

  return true;
}
#endif

/**
 * @brief Copy a buffer from user packet memory area (PMA) to FIFO.
 *        Uses byte-access of system memory and 16-bit access of packet memory
 * @param   wNBytes no. of bytes to be copied.
 * @retval None
 */
static bool dcd_read_packet_memory_ff(tu_fifo_t *ff, uint16_t src, uint16_t wNBytes) {
  // Since we copy into a ring buffer FIFO, a wrap might occur making it necessary to conduct two copies
  // Check for first linear part
  tu_fifo_buffer_info_t info;
  tu_fifo_get_write_info(ff, &info); // We want to read from the FIFO

  uint16_t cnt_lin = TU_MIN(wNBytes, info.len_lin);
  uint16_t cnt_wrap = TU_MIN(wNBytes - cnt_lin, info.len_wrap);

  // We want to read from PMA and write it into the FIFO, if LIN part is ODD and has WRAPPED part,
  // last lin byte will be combined with wrapped part
  // To ensure PMA is always access aligned (src aligned to 16 or 32 bit)
#ifdef FSDEV_BUS_32BIT
  if ((cnt_lin & 0x03) && cnt_wrap) {
    // Copy first linear part
    dcd_read_packet_memory(info.ptr_lin, src, cnt_lin & ~0x03);
    src += cnt_lin & ~0x03;

    // Copy last linear bytes & first wrapped bytes
    uint8_t tmp[4];
    dcd_read_packet_memory(tmp, src, 4);
    src += 4;

    uint32_t i;
    for (i = 0; i < (cnt_lin & 0x03); i++) {
      ((uint8_t *)info.ptr_lin)[(cnt_lin & ~0x03) + i] = tmp[i];
    }
    uint32_t wCnt = cnt_wrap;
    for (; i < 4 && wCnt > 0; i++, wCnt--) {
      *(uint8_t *)info.ptr_wrap = tmp[i];
      info.ptr_wrap = (uint8_t *)info.ptr_wrap + 1;
    }

    // Copy rest of wrapped byte
    if (wCnt) {
      dcd_read_packet_memory(info.ptr_wrap, src, wCnt);
    }
  }
#else
  if ((cnt_lin & 0x01) && cnt_wrap) {
    // Copy first linear part
    dcd_read_packet_memory(info.ptr_lin, src, cnt_lin & ~0x01);
    src += cnt_lin & ~0x01;

    // Copy last linear byte & first wrapped byte
    uint8_t tmp[2];
    dcd_read_packet_memory(tmp, src, 2);
    src += 2;

    ((uint8_t *)info.ptr_lin)[cnt_lin - 1] = tmp[0];
    ((uint8_t *)info.ptr_wrap)[0] = tmp[1];

    // Copy rest of wrapped byte
    dcd_read_packet_memory(((uint8_t *)info.ptr_wrap) + 1, src, cnt_wrap - 1);
  }
#endif
  else {
    // Copy linear part
    dcd_read_packet_memory(info.ptr_lin, src, cnt_lin);
    src += cnt_lin;

    if (info.len_wrap) {
      // Copy wrapped byte
      dcd_read_packet_memory(info.ptr_wrap, src, cnt_wrap);
    }
  }

  tu_fifo_advance_write_pointer(ff, cnt_lin + cnt_wrap);

  return true;
}

#endif
