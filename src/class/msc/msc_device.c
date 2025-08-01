/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#if (CFG_TUD_ENABLED && CFG_TUD_MSC)

#include "device/dcd.h"         // for faking dcd_event_xfer_complete
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "msc_device.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_MSC_LOG_LEVEL
  #define CFG_TUD_MSC_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_MSC_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// Weak stubs: invoked if no strong implementation is available
//--------------------------------------------------------------------+
TU_ATTR_WEAK void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
  (void) lun; (void) vendor_id; (void) product_id; (void) product_rev;
}
TU_ATTR_WEAK uint32_t tud_msc_inquiry2_cb(uint8_t lun, scsi_inquiry_resp_t *inquiry_resp, uint32_t bufsize) {
  (void) lun; (void) inquiry_resp; (void) bufsize;
  return 0;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
enum {
  MSC_STAGE_CMD  = 0,
  MSC_STAGE_DATA,
  MSC_STAGE_STATUS,
  MSC_STAGE_STATUS_SENT,
  MSC_STAGE_NEED_RESET,
};

typedef struct {
  TU_ATTR_ALIGNED(4) msc_cbw_t cbw; // 31 bytes
  uint8_t  rhport;

  TU_ATTR_ALIGNED(4) msc_csw_t csw; // 13 bytes
  uint8_t  itf_num;
  uint8_t  ep_in;
  uint8_t  ep_out;

  uint32_t total_len;   // byte to be transferred, can be smaller than total_bytes in cbw
  uint32_t xferred_len; // numbered of bytes transferred so far in the Data Stage

  // Bulk Only Transfer (BOT) Protocol
  uint8_t stage;

  // SCSI Sense Response Data
  uint8_t sense_key;
  uint8_t add_sense_code;
  uint8_t add_sense_qualifier;

  uint8_t pending_io; // pending async IO
}mscd_interface_t;

static mscd_interface_t _mscd_itf;

CFG_TUD_MEM_SECTION static struct {
  TUD_EPBUF_DEF(buf, CFG_TUD_MSC_EP_BUFSIZE);
} _mscd_epbuf;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static int32_t proc_builtin_scsi(uint8_t lun, uint8_t const scsi_cmd[16], uint8_t* buffer, uint32_t bufsize);
static void proc_read10_cmd(mscd_interface_t* p_msc);
static void proc_read_io_data(mscd_interface_t* p_msc, int32_t nbytes);
static void proc_write10_cmd(mscd_interface_t* p_msc);
static void proc_write10_host_data(mscd_interface_t* p_msc, uint32_t xferred_bytes);
static void proc_write_io_data(mscd_interface_t* p_msc, uint32_t xferred_bytes, int32_t nbytes);
static bool proc_stage_status(mscd_interface_t* p_msc);

TU_ATTR_ALWAYS_INLINE static inline bool is_data_in(uint8_t dir) {
  return tu_bit_test(dir, 7);
}

static inline bool send_csw(mscd_interface_t* p_msc) {
  // Data residue is always = host expect - actual transferred
  uint8_t rhport = p_msc->rhport;
  p_msc->csw.data_residue = p_msc->cbw.total_bytes - p_msc->xferred_len;
  p_msc->stage = MSC_STAGE_STATUS_SENT;
  memcpy(_mscd_epbuf.buf, &p_msc->csw, sizeof(msc_csw_t));
  return usbd_edpt_xfer(rhport, p_msc->ep_in , _mscd_epbuf.buf, sizeof(msc_csw_t));
}

static inline bool prepare_cbw(mscd_interface_t* p_msc) {
  uint8_t rhport = p_msc->rhport;
  p_msc->stage = MSC_STAGE_CMD;
  return usbd_edpt_xfer(rhport, p_msc->ep_out,  _mscd_epbuf.buf, sizeof(msc_cbw_t));
}

static void fail_scsi_op(mscd_interface_t* p_msc, uint8_t status) {
  msc_cbw_t const * p_cbw = &p_msc->cbw;
  msc_csw_t       * p_csw = &p_msc->csw;
  uint8_t rhport = p_msc->rhport;

  p_csw->status       = status;
  p_csw->data_residue = p_msc->cbw.total_bytes - p_msc->xferred_len;
  p_msc->stage        = MSC_STAGE_STATUS;

  // failed but sense key is not set: default to Illegal Request
  if (p_msc->sense_key == 0) {
    tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
  }

  // If there is data stage and not yet complete, stall it
  if (p_cbw->total_bytes && p_csw->data_residue) {
    if (is_data_in(p_cbw->dir)) {
      usbd_edpt_stall(rhport, p_msc->ep_in);
    } else {
      usbd_edpt_stall(rhport, p_msc->ep_out);
    }
  }
}

static inline uint32_t rdwr10_get_lba(uint8_t const command[]) {
  // use offsetof to avoid pointer to the odd/unaligned address
  const uint32_t lba = tu_unaligned_read32(command + offsetof(scsi_write10_t, lba));
  return tu_ntohl(lba); // lba is in Big Endian
}

static inline uint16_t rdwr10_get_blockcount(msc_cbw_t const* cbw) {
  uint16_t const block_count = tu_unaligned_read16(cbw->command + offsetof(scsi_write10_t, block_count));
  return tu_ntohs(block_count);
}

static inline uint16_t rdwr10_get_blocksize(msc_cbw_t const* cbw) {
  // first extract block count in the command
  uint16_t const block_count = rdwr10_get_blockcount(cbw);
  if (block_count == 0) {
    return 0; // invalid block count
  }
  return (uint16_t) (cbw->total_bytes / block_count);
}

static uint8_t rdwr10_validate_cmd(msc_cbw_t const* cbw) {
  uint8_t status = MSC_CSW_STATUS_PASSED;
  uint16_t const block_count = rdwr10_get_blockcount(cbw);

  if (cbw->total_bytes == 0) {
    if (block_count) {
      TU_LOG_DRV("  SCSI case 2 (Hn < Di) or case 3 (Hn < Do) \r\n");
      status = MSC_CSW_STATUS_PHASE_ERROR;
    } else {
      // no data transfer, only exist in complaint test suite
    }
  } else {
    if (SCSI_CMD_READ_10 == cbw->command[0] && !is_data_in(cbw->dir)) {
      TU_LOG_DRV("  SCSI case 10 (Ho <> Di)\r\n");
      status = MSC_CSW_STATUS_PHASE_ERROR;
    } else if (SCSI_CMD_WRITE_10 == cbw->command[0] && is_data_in(cbw->dir)) {
      TU_LOG_DRV("  SCSI case 8 (Hi <> Do)\r\n");
      status = MSC_CSW_STATUS_PHASE_ERROR;
    } else if (0 == block_count) {
      TU_LOG_DRV("  SCSI case 4 Hi > Dn (READ10) or case 9 Ho > Dn (WRITE10) \r\n");
      status = MSC_CSW_STATUS_FAILED;
    } else if (cbw->total_bytes / block_count == 0) {
      TU_LOG_DRV(" Computed block size = 0. SCSI case 7 Hi < Di (READ10) or case 13 Ho < Do (WRIT10)\r\n");
      status = MSC_CSW_STATUS_PHASE_ERROR;
    }
  }

  return status;
}

static bool proc_stage_status(mscd_interface_t *p_msc) {
  uint8_t rhport = p_msc->rhport;
  msc_cbw_t const *p_cbw = &p_msc->cbw;

  // skip status if epin is currently stalled, will do it when received Clear Stall request
  if (!usbd_edpt_stalled(rhport, p_msc->ep_in)) {
    if ((p_cbw->total_bytes > p_msc->xferred_len) && is_data_in(p_cbw->dir)) {
      // 6.7 The 13 Cases: case 5 (Hi > Di): STALL before status
      // TU_LOG_DRV("  SCSI case 5 (Hi > Di): %lu > %lu\r\n", p_cbw->total_bytes, p_msc->xferred_len);
      usbd_edpt_stall(rhport, p_msc->ep_in);
    } else {
      TU_ASSERT(send_csw(p_msc));
    }
  }

  #if TU_CHECK_MCU(OPT_MCU_CXD56)
  // WORKAROUND: cxd56 has its own nuttx usb stack which does not forward Set/ClearFeature(Endpoint) to DCD.
  // There is no way for us to know when EP is un-stall, therefore we will unconditionally un-stall here and
  // hope everything will work
  if (usbd_edpt_stalled(rhport, p_msc->ep_in)) {
    usbd_edpt_clear_stall(rhport, p_msc->ep_in);
    send_csw(p_msc);
  }
  #endif
  return true;
}

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_MSC_LOG_LEVEL

TU_ATTR_UNUSED tu_static tu_lookup_entry_t const _msc_scsi_cmd_lookup[] = {
  { .key = SCSI_CMD_TEST_UNIT_READY              , .data = "Test Unit Ready" },
  { .key = SCSI_CMD_INQUIRY                      , .data = "Inquiry" },
  { .key = SCSI_CMD_MODE_SELECT_6                , .data = "Mode_Select 6" },
  { .key = SCSI_CMD_MODE_SENSE_6                 , .data = "Mode_Sense 6" },
  { .key = SCSI_CMD_START_STOP_UNIT              , .data = "Start Stop Unit" },
  { .key = SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL , .data = "Prevent/Allow Medium Removal" },
  { .key = SCSI_CMD_READ_CAPACITY_10             , .data = "Read Capacity10" },
  { .key = SCSI_CMD_REQUEST_SENSE                , .data = "Request Sense" },
  { .key = SCSI_CMD_READ_FORMAT_CAPACITY         , .data = "Read Format Capacity" },
  { .key = SCSI_CMD_READ_10                      , .data = "Read10" },
  { .key = SCSI_CMD_WRITE_10                     , .data = "Write10" }
};

TU_ATTR_UNUSED tu_static tu_lookup_table_t const _msc_scsi_cmd_table = {
  .count = TU_ARRAY_SIZE(_msc_scsi_cmd_lookup),
  .items = _msc_scsi_cmd_lookup
};

#endif

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
bool tud_msc_set_sense(uint8_t lun, uint8_t sense_key, uint8_t add_sense_code, uint8_t add_sense_qualifier) {
  (void) lun;
  _mscd_itf.sense_key           = sense_key;
  _mscd_itf.add_sense_code      = add_sense_code;
  _mscd_itf.add_sense_qualifier = add_sense_qualifier;
  return true;
}

TU_ATTR_ALWAYS_INLINE static inline void set_sense_medium_not_present(uint8_t lun) {
  // default sense is NOT READY, MEDIUM NOT PRESENT
  tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
}

static void proc_async_io_done(void *bytes_io) {
  mscd_interface_t *p_msc = &_mscd_itf;
  TU_VERIFY(p_msc->pending_io, );
  const int32_t nbytes = (int32_t) (intptr_t) bytes_io;
  const uint8_t cmd = p_msc->cbw.command[0];

  p_msc->pending_io = 0;
  switch (cmd) {
    case SCSI_CMD_READ_10:
      proc_read_io_data(p_msc, nbytes);
      break;

    case SCSI_CMD_WRITE_10:
      proc_write_io_data(p_msc, (uint32_t) nbytes, nbytes);
      break;

    default: break;
  }

  // send status if stage is transitioned to STATUS
  if (p_msc->stage == MSC_STAGE_STATUS) {
    proc_stage_status(p_msc);
  }
}

bool tud_msc_async_io_done(int32_t bytes_io, bool in_isr) {
  // Precheck to avoid queueing multiple RW done callback
  TU_VERIFY(_mscd_itf.pending_io);
  if (bytes_io == 0) {
    bytes_io = TUD_MSC_RET_ERROR; // 0 is treated as error, no reason to call this with BUSY here
  }
  usbd_defer_func(proc_async_io_done, (void *) (intptr_t) bytes_io, in_isr);
  return true;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void mscd_init(void) {
  TU_LOG_INT(CFG_TUD_MSC_LOG_LEVEL, sizeof(mscd_interface_t));
  tu_memclr(&_mscd_itf, sizeof(mscd_interface_t));
}

bool mscd_deinit(void) {
  return true; // nothing to do
}

void mscd_reset(uint8_t rhport) {
  (void) rhport;
  tu_memclr(&_mscd_itf, sizeof(mscd_interface_t));
}

uint16_t mscd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len) {
  // only support SCSI's BOT protocol
  TU_VERIFY(TUSB_CLASS_MSC    == itf_desc->bInterfaceClass &&
            MSC_SUBCLASS_SCSI == itf_desc->bInterfaceSubClass &&
            MSC_PROTOCOL_BOT  == itf_desc->bInterfaceProtocol, 0);
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + 2*sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(max_len >= drv_len, 0); // Max length must be at least 1 interface + 2 endpoints

  mscd_interface_t * p_msc = &_mscd_itf;
  p_msc->itf_num = itf_desc->bInterfaceNumber;
  p_msc->rhport = rhport;

  // Open endpoint pair
  TU_ASSERT(usbd_open_edpt_pair(rhport, tu_desc_next(itf_desc), 2, TUSB_XFER_BULK, &p_msc->ep_out, &p_msc->ep_in), 0);

  // Prepare for Command Block Wrapper
  TU_ASSERT(prepare_cbw(p_msc), drv_len);

  return drv_len;
}

static void proc_bot_reset(mscd_interface_t* p_msc) {
  p_msc->stage       = MSC_STAGE_CMD;
  p_msc->total_len   = 0;
  p_msc->xferred_len = 0;
  p_msc->sense_key           = 0;
  p_msc->add_sense_code      = 0;
  p_msc->add_sense_qualifier = 0;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool mscd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
  if (stage != CONTROL_STAGE_SETUP) {
    return true; // nothing to do with DATA & ACK stage
  }

  mscd_interface_t* p_msc = &_mscd_itf;

  // Clear Endpoint Feature (stall) for recovery
  if ( TUSB_REQ_TYPE_STANDARD     == request->bmRequestType_bit.type      &&
       TUSB_REQ_RCPT_ENDPOINT     == request->bmRequestType_bit.recipient &&
       TUSB_REQ_CLEAR_FEATURE     == request->bRequest                    &&
       TUSB_REQ_FEATURE_EDPT_HALT == request->wValue ) {
    uint8_t const ep_addr = tu_u16_low(request->wIndex);

    if (p_msc->stage == MSC_STAGE_NEED_RESET) {
      // reset recovery is required to recover from this stage
      // Clear Stall request cannot resolve this -> continue to stall endpoint
      usbd_edpt_stall(rhport, ep_addr);
    } else {
      if (ep_addr == p_msc->ep_in) {
        if (p_msc->stage == MSC_STAGE_STATUS) {
          // resume sending SCSI status if we are in this stage previously before stalled
          TU_ASSERT(send_csw(p_msc));
        }
      } else if (ep_addr == p_msc->ep_out) {
        if (p_msc->stage == MSC_STAGE_CMD) {
          // part of reset recovery (probably due to invalid CBW) -> prepare for new command
          // Note: skip if already queued previously
          if (usbd_edpt_ready(rhport, p_msc->ep_out)) {
            TU_ASSERT(prepare_cbw(p_msc));
          }
        }
      }
    }

    return true;
  }

  // From this point only handle class request only
  TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  switch ( request->bRequest ) {
    case MSC_REQ_RESET:
      TU_LOG_DRV("  MSC BOT Reset\r\n");
      TU_VERIFY(request->wValue == 0 && request->wLength == 0);
      proc_bot_reset(p_msc); // driver state reset
      tud_control_status(rhport, request);
    break;

    case MSC_REQ_GET_MAX_LUN: {
      TU_LOG_DRV("  MSC Get Max Lun\r\n");
      TU_VERIFY(request->wValue == 0 && request->wLength == 1);

      uint8_t maxlun = 1;
      if (tud_msc_get_maxlun_cb) {
        maxlun = tud_msc_get_maxlun_cb();
      }
      TU_VERIFY(maxlun);
      maxlun--; // MAX LUN is minus 1 by specs
      tud_control_xfer(rhport, request, &maxlun, 1);
      break;
    }

    default: return false; // stall unsupported request
  }

  return true;
}

bool mscd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes) {
  (void) event;

  mscd_interface_t* p_msc = &_mscd_itf;
  msc_cbw_t * p_cbw = &p_msc->cbw;
  msc_csw_t * p_csw = &p_msc->csw;

  switch (p_msc->stage) {
    case MSC_STAGE_CMD: {
      //------------- new CBW received -------------//
      // Complete IN while waiting for CMD is usually Status of previous SCSI op, ignore it
      if (ep_addr != p_msc->ep_out) {
        return true;
      }

      const uint32_t signature = tu_le32toh(tu_unaligned_read32(_mscd_epbuf.buf));

      if (!(xferred_bytes == sizeof(msc_cbw_t) && signature == MSC_CBW_SIGNATURE)) {
        // BOT 6.6.1 If CBW is not valid stall both endpoints until reset recovery
        TU_LOG_DRV("  SCSI CBW is not valid\r\n");
        p_msc->stage = MSC_STAGE_NEED_RESET;
        usbd_edpt_stall(rhport, p_msc->ep_in);
        usbd_edpt_stall(rhport, p_msc->ep_out);
        return false;
      }

      memcpy(p_cbw, _mscd_epbuf.buf, sizeof(msc_cbw_t));

      TU_LOG_DRV("  SCSI Command [Lun%u]: %s\r\n", p_cbw->lun, tu_lookup_find(&_msc_scsi_cmd_table, p_cbw->command[0]));
      // TU_LOG_MEM(CFG_TUD_MSC_LOG_LEVEL, p_cbw, xferred_bytes, 2);

      p_csw->signature    = MSC_CSW_SIGNATURE;
      p_csw->tag          = p_cbw->tag;
      p_csw->data_residue = 0;
      p_csw->status       = MSC_CSW_STATUS_PASSED;

      /*------------- Parse command and prepare DATA -------------*/
      p_msc->stage = MSC_STAGE_DATA;
      p_msc->total_len = p_cbw->total_bytes;
      p_msc->xferred_len = 0;

      // Read10 or Write10
      if ((SCSI_CMD_READ_10 == p_cbw->command[0]) || (SCSI_CMD_WRITE_10 == p_cbw->command[0])) {
        uint8_t const status = rdwr10_validate_cmd(p_cbw);

        if (status != MSC_CSW_STATUS_PASSED) {
          fail_scsi_op(p_msc, status);
        } else if (p_cbw->total_bytes) {
          if (SCSI_CMD_READ_10 == p_cbw->command[0]) {
            proc_read10_cmd(p_msc);
          } else {
            proc_write10_cmd(p_msc);
          }
        } else {
          // no data transfer, only exist in complaint test suite
          p_msc->stage = MSC_STAGE_STATUS;
        }
      } else {
        // For other SCSI commands
        // 1. OUT : queue transfer (invoke app callback after done)
        // 2. IN & Zero: Process if is built-in, else Invoke app callback. Skip DATA if zero length
        if ((p_cbw->total_bytes > 0) && !is_data_in(p_cbw->dir)) {
          if (p_cbw->total_bytes > CFG_TUD_MSC_EP_BUFSIZE) {
            TU_LOG_DRV("  SCSI reject non READ10/WRITE10 with large data\r\n");
            fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
          } else {
            // Didn't check for case 9 (Ho > Dn), which requires examining scsi command first
            // but it is OK to just receive data then responded with failed status
            TU_ASSERT(usbd_edpt_xfer(rhport, p_msc->ep_out, _mscd_epbuf.buf, (uint16_t) p_msc->total_len));
          }
        } else {
          // First process if it is a built-in commands
          int32_t resplen = proc_builtin_scsi(p_cbw->lun, p_cbw->command, _mscd_epbuf.buf, CFG_TUD_MSC_EP_BUFSIZE);

          // Invoke user callback if not built-in
          if ((resplen < 0) && (p_msc->sense_key == 0)) {
            resplen = tud_msc_scsi_cb(p_cbw->lun, p_cbw->command, _mscd_epbuf.buf, (uint16_t)p_msc->total_len);
          }

          if (resplen < 0) {
            // unsupported command
            TU_LOG_DRV("  SCSI unsupported or failed command\r\n");
            fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
          } else if (resplen == 0) {
            if (p_cbw->total_bytes) {
              // 6.7 The 13 Cases: case 4 (Hi > Dn)
              // TU_LOG_DRV("  SCSI case 4 (Hi > Dn): %lu\r\n", p_cbw->total_bytes);
              fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
            } else {
              // case 1 Hn = Dn: all good
              p_msc->stage = MSC_STAGE_STATUS;
            }
          } else {
            if (p_cbw->total_bytes == 0) {
              // 6.7 The 13 Cases: case 2 (Hn < Di)
              // TU_LOG_DRV("  SCSI case 2 (Hn < Di): %lu\r\n", p_cbw->total_bytes);
              fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
            } else {
              // cannot return more than host expect
              p_msc->total_len = tu_min32((uint32_t)resplen, p_cbw->total_bytes);
              TU_ASSERT(usbd_edpt_xfer(rhport, p_msc->ep_in, _mscd_epbuf.buf, (uint16_t) p_msc->total_len));
            }
          }
        }
      }
      break;
    }

    case MSC_STAGE_DATA:
      TU_LOG_DRV("  SCSI Data [Lun%u]\r\n", p_cbw->lun);
      TU_ASSERT(xferred_bytes <= CFG_TUD_MSC_EP_BUFSIZE); // sanity check to avoid buffer overflow
      // TU_LOG_MEM(CFG_TUD_MSC_LOG_LEVEL, _mscd_epbuf.buf, xferred_bytes, 2);

      if (SCSI_CMD_READ_10 == p_cbw->command[0]) {
        p_msc->xferred_len += xferred_bytes;

        if ( p_msc->xferred_len >= p_msc->total_len ) {
          // Data Stage is complete
          p_msc->stage = MSC_STAGE_STATUS;
        }else {
          proc_read10_cmd(p_msc);
        }
      } else if (SCSI_CMD_WRITE_10 == p_cbw->command[0]) {
        proc_write10_host_data(p_msc, xferred_bytes);
      } else {
        p_msc->xferred_len += xferred_bytes;

        // OUT transfer, invoke callback if needed
        if ( !is_data_in(p_cbw->dir) ) {
          int32_t cb_result = tud_msc_scsi_cb(p_cbw->lun, p_cbw->command, _mscd_epbuf.buf, (uint16_t) p_msc->total_len);

          if ( cb_result < 0 ) {
            // unsupported command
            TU_LOG_DRV("  SCSI unsupported command\r\n");
            fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
          }else {
            // TODO haven't implement this scenario any further yet
          }
        }

        if ( p_msc->xferred_len >= p_msc->total_len ) {
          // Data Stage is complete
          p_msc->stage = MSC_STAGE_STATUS;
        } else {
          // This scenario with command that take more than one transfer is already rejected at Command stage
          TU_BREAKPOINT();
        }
      }
    break;

    case MSC_STAGE_STATUS:
      // processed immediately after this switch, supposedly to be empty
    break;

    case MSC_STAGE_STATUS_SENT:
      // Status phase is complete
      if ((ep_addr == p_msc->ep_in) && (xferred_bytes == sizeof(msc_csw_t))) {
        TU_LOG_DRV("  SCSI Status [Lun%u] = %u\r\n", p_cbw->lun, p_csw->status);
        // TU_LOG_MEM(CFG_TUD_MSC_LOG_LEVEL, p_csw, xferred_bytes, 2);

        // Invoke complete callback if defined
        // Note: There is racing issue with samd51 + qspi flash testing with arduino
        // if complete_cb() is invoked after queuing the status.
        switch (p_cbw->command[0]) {
          case SCSI_CMD_READ_10:
            if (tud_msc_read10_complete_cb) {
              tud_msc_read10_complete_cb(p_cbw->lun);
            }
            break;

          case SCSI_CMD_WRITE_10:
            if (tud_msc_write10_complete_cb) {
              tud_msc_write10_complete_cb(p_cbw->lun);
            }
            break;

          default:
            if (tud_msc_scsi_complete_cb) {
              tud_msc_scsi_complete_cb(p_cbw->lun, p_cbw->command);
            }
            break;
        }

        TU_ASSERT(prepare_cbw(p_msc));
      } else {
        // Any xfer ended here is considered unknown error, ignore it
        TU_LOG1("  Warning expect SCSI Status but received unknown data\r\n");
      }
      break;

    default: break;
  }

  if (p_msc->stage == MSC_STAGE_STATUS) {
    TU_ASSERT(proc_stage_status(p_msc));
  }

  return true;
}

/*------------------------------------------------------------------*/
/* SCSI Command Process
 *------------------------------------------------------------------*/

// return response's length (copied to buffer). Negative if it is not an built-in command or indicate Failed status (CSW)
// In case of a failed status, sense key must be set for reason of failure
static int32_t proc_builtin_scsi(uint8_t lun, uint8_t const scsi_cmd[16], uint8_t* buffer, uint32_t bufsize) {
  (void)bufsize; // TODO refractor later
  int32_t resplen;

  mscd_interface_t* p_msc = &_mscd_itf;

  switch (scsi_cmd[0]) {
    case SCSI_CMD_TEST_UNIT_READY:
      resplen = 0;
      if (!tud_msc_test_unit_ready_cb(lun)) {
        // Failed status response
        resplen = -1;

        // set default sense if not set by callback
        if (p_msc->sense_key == 0) {
          set_sense_medium_not_present(lun);
        }
      }
      break;

    case SCSI_CMD_START_STOP_UNIT:
      resplen = 0;

      if (tud_msc_start_stop_cb) {
        scsi_start_stop_unit_t const* start_stop = (scsi_start_stop_unit_t const*)scsi_cmd;
        if (!tud_msc_start_stop_cb(lun, start_stop->power_condition, start_stop->start, start_stop->load_eject)) {
          // Failed status response
          resplen = -1;

          // set default sense if not set by callback
          if (p_msc->sense_key == 0) {
            set_sense_medium_not_present(lun);
          }
        }
      }
      break;

    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      resplen = 0;

      if (tud_msc_prevent_allow_medium_removal_cb) {
        scsi_prevent_allow_medium_removal_t const* prevent_allow = (scsi_prevent_allow_medium_removal_t const*)scsi_cmd;
        if (!tud_msc_prevent_allow_medium_removal_cb(lun, prevent_allow->prohibit_removal, prevent_allow->control)) {
          // Failed status response
          resplen = -1;

          // set default sense if not set by callback
          if (p_msc->sense_key == 0) {
            set_sense_medium_not_present(lun);
          }
        }
      }
      break;


    case SCSI_CMD_READ_CAPACITY_10: {
      uint32_t block_count;
      uint32_t block_size;
      uint16_t block_size_u16;

      tud_msc_capacity_cb(lun, &block_count, &block_size_u16);
      block_size = (uint32_t)block_size_u16;

      // Invalid block size/count from callback, possibly unit is not ready
      // stall this request, set sense key to NOT READY
      if (block_count == 0 || block_size == 0) {
        resplen = -1;

        // set default sense if not set by callback
        if (p_msc->sense_key == 0) {
          set_sense_medium_not_present(lun);
        }
      } else {
        scsi_read_capacity10_resp_t read_capa10;

        read_capa10.last_lba = tu_htonl(block_count-1);
        read_capa10.block_size = tu_htonl(block_size);

        resplen = sizeof(read_capa10);
        TU_VERIFY(0 == tu_memcpy_s(buffer, bufsize, &read_capa10, (size_t) resplen));
      }
    }
    break;

    case SCSI_CMD_READ_FORMAT_CAPACITY: {
      scsi_read_format_capacity_data_t read_fmt_capa = {
        .list_length = 8,
        .block_num = 0,
        .descriptor_type = 2, // formatted media
        .block_size_u16 = 0
      };

      uint32_t block_count;
      uint16_t block_size;

      tud_msc_capacity_cb(lun, &block_count, &block_size);

      // Invalid block size/count from callback, possibly unit is not ready
      // stall this request, set sense key to NOT READY
      if (block_count == 0 || block_size == 0) {
        resplen = -1;

        // set default sense if not set by callback
        if (p_msc->sense_key == 0) {
          set_sense_medium_not_present(lun);
        }
      } else {
        read_fmt_capa.block_num = tu_htonl(block_count);
        read_fmt_capa.block_size_u16 = tu_htons(block_size);

        resplen = sizeof(read_fmt_capa);
        TU_VERIFY(0 == tu_memcpy_s(buffer, bufsize, &read_fmt_capa, (size_t) resplen));
      }
    }
    break;

    case SCSI_CMD_INQUIRY: {
      scsi_inquiry_resp_t *inquiry_rsp = (scsi_inquiry_resp_t *) buffer;
      tu_memclr(inquiry_rsp, sizeof(scsi_inquiry_resp_t));
      inquiry_rsp->is_removable = 1;
      inquiry_rsp->version = 2;
      inquiry_rsp->response_data_format = 2;
      inquiry_rsp->additional_length = sizeof(scsi_inquiry_resp_t) - 5;

      resplen = (int32_t) tud_msc_inquiry2_cb(lun, inquiry_rsp, bufsize);
      if (resplen == 0) {
        // stub callback with no response, use v1 callback
        tud_msc_inquiry_cb(lun, inquiry_rsp->vendor_id, inquiry_rsp->product_id, inquiry_rsp->product_rev);
        resplen = sizeof(scsi_inquiry_resp_t);
      }
    }
    break;

    case SCSI_CMD_MODE_SENSE_6: {
      scsi_mode_sense6_resp_t mode_resp = {
        .data_len = 3,
        .medium_type = 0,
        .write_protected = false,
        .reserved = 0,
        .block_descriptor_len = 0 // no block descriptor are included
      };

      bool writable = true;
      if (tud_msc_is_writable_cb) {
        writable = tud_msc_is_writable_cb(lun);
      }

      mode_resp.write_protected = !writable;

      resplen = sizeof(mode_resp);
      TU_VERIFY(0 == tu_memcpy_s(buffer, bufsize, &mode_resp, (size_t) resplen));
    }
    break;

    case SCSI_CMD_REQUEST_SENSE: {
      scsi_sense_fixed_resp_t sense_rsp = {
        .response_code = 0x70, // current, fixed format
        .valid = 1
      };

      sense_rsp.add_sense_len = sizeof(scsi_sense_fixed_resp_t) - 8;
      sense_rsp.sense_key = (uint8_t)(p_msc->sense_key & 0x0F);
      sense_rsp.add_sense_code = p_msc->add_sense_code;
      sense_rsp.add_sense_qualifier = p_msc->add_sense_qualifier;

      resplen = sizeof(sense_rsp);
      TU_VERIFY(0 == tu_memcpy_s(buffer, bufsize, &sense_rsp, (size_t) resplen));

      // request sense callback could overwrite the sense data
      if (tud_msc_request_sense_cb) {
        resplen = tud_msc_request_sense_cb(lun, buffer, (uint16_t)bufsize);
      }

      // Clear sense data after copy
      tud_msc_set_sense(lun, 0, 0, 0);
    }
    break;

    default: resplen = -1;
      break;
  }

  return resplen;
}

static void proc_read10_cmd(mscd_interface_t* p_msc) {
  msc_cbw_t const* p_cbw = &p_msc->cbw;
  uint16_t const block_sz = rdwr10_get_blocksize(p_cbw); // already verified non-zero
  // Adjust lba & offset with transferred bytes
  uint32_t const lba = rdwr10_get_lba(p_cbw->command) + (p_msc->xferred_len / block_sz);
  uint32_t const offset = p_msc->xferred_len % block_sz;

  // remaining bytes capped at class buffer
  int32_t nbytes = (int32_t)tu_min32(CFG_TUD_MSC_EP_BUFSIZE, p_cbw->total_bytes - p_msc->xferred_len);

  p_msc->pending_io = 1;
  nbytes = tud_msc_read10_cb(p_cbw->lun, lba, offset, _mscd_epbuf.buf, (uint32_t)nbytes);
  if (nbytes != TUD_MSC_RET_ASYNC) {
    p_msc->pending_io = 0;
    proc_read_io_data(p_msc, nbytes);
  }
}

static void proc_read_io_data(mscd_interface_t* p_msc, int32_t nbytes) {
  const uint8_t rhport = p_msc->rhport;
  if (nbytes > 0) {
    TU_ASSERT(usbd_edpt_xfer(rhport, p_msc->ep_in, _mscd_epbuf.buf, (uint16_t) nbytes),);
  } else {
    // nbytes is status
    switch (nbytes) {
      case TUD_MSC_RET_ERROR:
        // error -> endpoint is stalled & status in CSW set to failed
        TU_LOG_DRV("  IO read() failed\r\n");
        set_sense_medium_not_present(p_msc->cbw.lun);
        fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
        break;

      case TUD_MSC_RET_BUSY:
        // not ready yet -> fake a transfer complete so that this driver callback will fire again
        dcd_event_xfer_complete(rhport, p_msc->ep_in, 0, XFER_RESULT_SUCCESS, false);
        break;

      default: break;
    }
  }
}

static void proc_write10_cmd(mscd_interface_t* p_msc) {
  msc_cbw_t const* p_cbw = &p_msc->cbw;
  bool writable = true;

  if (tud_msc_is_writable_cb) {
    writable = tud_msc_is_writable_cb(p_cbw->lun);
  }

  if (!writable) {
    // Not writable, complete this SCSI op with error
    // Sense = Write protected
    tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00);
    fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
    return;
  }

  // remaining bytes capped at class buffer
  uint16_t nbytes = (uint16_t)tu_min32(CFG_TUD_MSC_EP_BUFSIZE, p_cbw->total_bytes - p_msc->xferred_len);
  // Write10 callback will be called later when usb transfer complete
  TU_ASSERT(usbd_edpt_xfer(p_msc->rhport, p_msc->ep_out, _mscd_epbuf.buf, nbytes),);
}

// process new data arrived from WRITE10
static void proc_write10_host_data(mscd_interface_t* p_msc, uint32_t xferred_bytes) {
  msc_cbw_t const* p_cbw = &p_msc->cbw;
  uint16_t const block_sz = rdwr10_get_blocksize(p_cbw); // already verified non-zero

  // Adjust lba & offset with transferred bytes
  uint32_t const lba = rdwr10_get_lba(p_cbw->command) + (p_msc->xferred_len / block_sz);
  uint32_t const offset = p_msc->xferred_len % block_sz;

  p_msc->pending_io = 1;
  int32_t nbytes =  tud_msc_write10_cb(p_cbw->lun, lba, offset, _mscd_epbuf.buf, xferred_bytes);
  if (nbytes != TUD_MSC_RET_ASYNC) {
    p_msc->pending_io = 0;
    proc_write_io_data(p_msc, xferred_bytes, nbytes);
  }
}

static void proc_write_io_data(mscd_interface_t* p_msc, uint32_t xferred_bytes, int32_t nbytes) {
  if (nbytes < 0) {
    // nbytes is status
    switch (nbytes) {
      case TUD_MSC_RET_ERROR:
        // IO error -> failed this scsi op
        TU_LOG_DRV("  IO write() failed\r\n");
        set_sense_medium_not_present(p_msc->cbw.lun);
        fail_scsi_op(p_msc, MSC_CSW_STATUS_FAILED);
        break;

      default: break;
    }
  } else {
    if ((uint32_t)nbytes < xferred_bytes) {
      // Application consume less than what we got including TUD_MSC_RET_BUSY (0)
      const uint32_t left_over = xferred_bytes - (uint32_t)nbytes;
      if (nbytes > 0) {
        memmove(_mscd_epbuf.buf, _mscd_epbuf.buf + nbytes, left_over);
      }

      // fake a transfer complete with adjusted parameters --> callback will be invoked with adjusted parameters
      dcd_event_xfer_complete(p_msc->rhport, p_msc->ep_out, left_over, XFER_RESULT_SUCCESS, false);
    } else {
      // Application consume all bytes in our buffer
      p_msc->xferred_len += xferred_bytes;

      if (p_msc->xferred_len >= p_msc->total_len) {
        // Data Stage is complete
        p_msc->stage = MSC_STAGE_STATUS;
      } else {
        // prepare to receive more data from host
        proc_write10_cmd(p_msc);
      }
    }
  }
}

#endif
