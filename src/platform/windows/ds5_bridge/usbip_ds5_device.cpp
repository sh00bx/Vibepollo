/**
 * @file src/platform/windows/ds5_bridge/usbip_ds5_device.cpp
 * @brief Multi-slot USB/IP DualSense device emulation (see header). Standard
 *        Linux USB/IP protocol (version 0x0111), the dialect the
 *        Microsoft-signed usbip-win2 vhci client speaks.
 *
 * License-clean: derived from our own host half (apollo 8f6b61e0 + M1 composite
 * descriptor). No CTM source consulted.
 */
// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>

#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/usbip_ds5_device.h"

using namespace std::literals;

namespace platf::ds5_bridge {

  namespace {

    constexpr uint16_t USBIP_VERSION = 0x0111;
    constexpr uint16_t OP_REQ_DEVLIST = 0x8005;
    constexpr uint16_t OP_REP_DEVLIST = 0x0005;
    constexpr uint16_t OP_REQ_IMPORT = 0x8003;
    constexpr uint16_t OP_REP_IMPORT = 0x0003;

    constexpr uint32_t CMD_SUBMIT = 0x0001;
    constexpr uint32_t RET_SUBMIT = 0x0003;
    constexpr uint32_t CMD_UNLINK = 0x0002;
    constexpr uint32_t RET_UNLINK = 0x0004;

    constexpr uint32_t DIR_OUT = 0;
    constexpr uint32_t DIR_IN = 1;

    // ---- DualSense descriptors (real DS5, VID 054C / PID 0CE6) -------------
    // iSerialNumber = 3 (string #3 is built per-slot from the pad serial).
    const uint8_t DEVICE_DESC[18] = {
      0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
      0x4C, 0x05, 0xE6, 0x0C, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
    };

    // Full 227-byte composite descriptor (3 Audio UAC1 + 1 HID), verbatim from a
    // real DS5. iso-OUT EP 0x01 (4ch/16/48k, wMaxPacketSize 392, bInterval 4);
    // HID interface = 3, HID class descriptor @ off 204.
    const uint8_t CONFIG_DESC[227] = {
      0x09,0x02,0xE3,0x00,0x04,0x01,0x00,0xC0,0xFA,0x09,0x04,0x00,0x00,0x00,0x01,0x01,
      0x00,0x00,0x0A,0x24,0x01,0x00,0x01,0x49,0x00,0x02,0x01,0x02,0x0C,0x24,0x02,0x01,
      0x01,0x01,0x06,0x04,0x33,0x00,0x00,0x00,0x0C,0x24,0x06,0x02,0x01,0x01,0x03,0x00,
      0x00,0x00,0x00,0x00,0x09,0x24,0x03,0x03,0x01,0x03,0x04,0x02,0x00,0x0C,0x24,0x02,
      0x04,0x02,0x04,0x03,0x02,0x03,0x00,0x00,0x00,0x09,0x24,0x06,0x05,0x04,0x01,0x03,
      0x00,0x00,0x09,0x24,0x03,0x06,0x01,0x01,0x01,0x05,0x00,0x09,0x04,0x01,0x00,0x00,
      0x01,0x02,0x00,0x00,0x09,0x04,0x01,0x01,0x01,0x01,0x02,0x00,0x00,0x07,0x24,0x01,
      0x01,0x01,0x01,0x00,0x0B,0x24,0x02,0x01,0x04,0x02,0x10,0x01,0x80,0xBB,0x00,0x09,
      0x05,0x01,0x09,0x88,0x01,0x04,0x00,0x00,0x07,0x25,0x01,0x00,0x00,0x00,0x00,0x09,
      0x04,0x02,0x00,0x00,0x01,0x02,0x00,0x00,0x09,0x04,0x02,0x01,0x01,0x01,0x02,0x00,
      0x00,0x07,0x24,0x01,0x06,0x01,0x01,0x00,0x0B,0x24,0x02,0x01,0x02,0x02,0x10,0x01,
      0x80,0xBB,0x00,0x09,0x05,0x82,0x05,0xC4,0x00,0x04,0x00,0x00,0x07,0x25,0x01,0x00,
      0x00,0x00,0x00,0x09,0x04,0x03,0x00,0x02,0x03,0x00,0x00,0x00,0x09,0x21,0x11,0x01,
      0x00,0x01,0x22,0x11,0x01,0x07,0x05,0x84,0x03,0x40,0x00,0x06,0x07,0x05,0x03,0x03,
      0x40,0x00,0x06,
    };

    // Real 273-byte DS5 HID report descriptor (no vendor reports — clean).
    const uint8_t HID_REPORT_DESC[273] = {
      0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35,
      0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06,
      0x00, 0xFF, 0x09, 0x20, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
      0x35, 0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05,
      0x09, 0x19, 0x01, 0x29, 0x0F, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0F, 0x81, 0x02, 0x06,
      0x00, 0xFF, 0x09, 0x21, 0x95, 0x0D, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x22, 0x15, 0x00, 0x26,
      0xFF, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23, 0x95, 0x2F, 0x91, 0x02,
      0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xB1, 0x02, 0x85, 0x08, 0x09, 0x34, 0x95, 0x2F, 0xB1, 0x02,
      0x85, 0x09, 0x09, 0x24, 0x95, 0x13, 0xB1, 0x02, 0x85, 0x0A, 0x09, 0x25, 0x95, 0x1A, 0xB1, 0x02,
      0x85, 0x20, 0x09, 0x26, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x21, 0x09, 0x27, 0x95, 0x04, 0xB1, 0x02,
      0x85, 0x22, 0x09, 0x40, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x80, 0x09, 0x28, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0x81, 0x09, 0x29, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x82, 0x09, 0x2A, 0x95, 0x09, 0xB1, 0x02,
      0x85, 0x83, 0x09, 0x2B, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x84, 0x09, 0x2C, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0x85, 0x09, 0x2D, 0x95, 0x02, 0xB1, 0x02, 0x85, 0xA0, 0x09, 0x2E, 0x95, 0x01, 0xB1, 0x02,
      0x85, 0xE0, 0x09, 0x2F, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF0, 0x09, 0x30, 0x95, 0x3F, 0xB1, 0x02,
      0x85, 0xF1, 0x09, 0x31, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF2, 0x09, 0x32, 0x95, 0x0F, 0xB1, 0x02,
      0x85, 0xF4, 0x09, 0x35, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF5, 0x09, 0x36, 0x95, 0x03, 0xB1, 0x02,
      0xC0,
    };

    // Genuine feature reports captured from a real DualSense (fallback when the
    // slot has no live feature provider). All 64 bytes.
    const uint8_t FEAT_05[64] = {
      0x05,0xFC,0xFF,0xF3,0xFF,0xFC,0xFF,0x99,0x22,0x5F,0xDD,0x8C,0x22,0x59,0xDD,0xBC,
      0x22,0x3E,0xDD,0x1C,0x02,0x1C,0x02,0x15,0x20,0x15,0xE0,0xB5,0x1F,0xDD,0xDF,0xFD,
      0x1F,0xFE,0xDF,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_09[64] = {
      0x09,0x0F,0xB1,0xA9,0xC1,0xBC,0xD0,0x08,0x25,0x00,0xB2,0xE5,0xBE,0x21,0xE4,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_20[64] = {
      0x20,0x4A,0x75,0x6E,0x20,0x32,0x34,0x20,0x32,0x30,0x32,0x34,0x31,0x31,0x3A,0x31,
      0x36,0x3A,0x32,0x31,0x02,0x00,0x04,0x00,0x13,0x03,0x00,0x00,0x00,0x00,0x0F,0x01,
      0x41,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x05,0x00,0x00,
      0x2A,0x00,0x01,0x00,0x0A,0x00,0x02,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    };
    const uint8_t FEAT_81[64] = {0x81};

    // ---- big-endian write helpers -----------------------------------------
    void put32(std::vector<uint8_t> &v, uint32_t x) {
      v.push_back((x >> 24) & 0xFF); v.push_back((x >> 16) & 0xFF);
      v.push_back((x >> 8) & 0xFF);  v.push_back(x & 0xFF);
    }
    void put16(std::vector<uint8_t> &v, uint16_t x) {
      v.push_back((x >> 8) & 0xFF); v.push_back(x & 0xFF);
    }
    uint32_t rd32(const uint8_t *p) {
      return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
    }
    uint16_t le16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }

    // Build a USB string descriptor (UTF-16LE) from an ASCII source.
    void build_string_desc(const std::string &s, std::vector<uint8_t> &out) {
      size_t n = std::min<size_t>(s.size(), 30);
      out.clear();
      out.push_back((uint8_t) (2 + n * 2));
      out.push_back(0x03);
      for (size_t i = 0; i < n; ++i) { out.push_back((uint8_t) s[i]); out.push_back(0x00); }
    }

    // usbip_usb_device wire record (big-endian) for DEVLIST / IMPORT replies.
    void append_usb_device(std::vector<uint8_t> &v, const std::string &busid, bool with_iface) {
      uint8_t path[256] = {0};
      std::string p = "/sys/devices/usbip/" + busid;
      std::strncpy((char *) path, p.c_str(), sizeof(path) - 1);
      uint8_t bid[32] = {0};
      std::strncpy((char *) bid, busid.c_str(), sizeof(bid) - 1);
      v.insert(v.end(), path, path + 256);
      v.insert(v.end(), bid, bid + 32);
      put32(v, 1); put32(v, 1); put32(v, 3);       // busnum, devnum, speed(3=high, for iso audio)
      put16(v, 0x054C); put16(v, 0x0CE6); put16(v, 0x0100);
      v.push_back(0x00); v.push_back(0x00); v.push_back(0x00);  // dev class/sub/proto
      v.push_back(0x01); v.push_back(0x01); v.push_back(0x04);  // cfgval, nconf, nif (4 ifaces)
      if (with_iface) {
        v.push_back(0x01); v.push_back(0x01); v.push_back(0x00); v.push_back(0x00);  // 0 AudioControl
        v.push_back(0x01); v.push_back(0x02); v.push_back(0x00); v.push_back(0x00);  // 1 AudioStreaming
        v.push_back(0x01); v.push_back(0x02); v.push_back(0x00); v.push_back(0x00);  // 2 AudioStreaming
        v.push_back(0x03); v.push_back(0x00); v.push_back(0x00); v.push_back(0x00);  // 3 HID
      }
    }

    // ------------------------------------------------------------------------
    //  Per-session state machine — one live vhci import bound to one slot.
    // ------------------------------------------------------------------------
    struct session_t {
      SOCKET sock;
      std::shared_ptr<slot_t> slot;
      std::mutex send_mtx;
      std::mutex pend_mtx;
      std::deque<std::pair<uint32_t, uint32_t>> pending_in;  // (seqnum, devid)
      std::atomic<bool> stop {false};

      bool send_all(const uint8_t *buf, int len) {
        std::lock_guard<std::mutex> lk(send_mtx);
        int off = 0;
        while (off < len) {
          int n = ::send(sock, (const char *) buf + off, len - off, 0);
          if (n <= 0) return false;
          off += n;
        }
        return true;
      }

      bool recv_n(uint8_t *buf, int n) {
        int off = 0;
        while (off < n) {
          int r = ::recv(sock, (char *) buf + off, n - off, 0);
          if (r <= 0) return false;
          off += r;
        }
        return true;
      }

      void ret_submit(uint32_t seqnum, uint32_t devid, uint32_t direction, uint32_t ep,
                      int32_t status, const uint8_t *payload, int paylen) {
        std::vector<uint8_t> v;
        v.reserve(48 + (direction == DIR_IN ? paylen : 0));
        put32(v, RET_SUBMIT); put32(v, seqnum); put32(v, devid);
        put32(v, direction);  put32(v, ep);
        put32(v, (uint32_t) status);
        put32(v, (uint32_t) (direction == DIR_IN ? paylen : 0));
        put32(v, 0); put32(v, 0); put32(v, 0);     // start_frame, npkts, error_count
        for (int i = 0; i < 8; i++) v.push_back(0);
        if (direction == DIR_IN && paylen > 0)
          v.insert(v.end(), payload, payload + paylen);
        send_all(v.data(), (int) v.size());
      }

      // ISO transfer reply — actual_length + npkts + echoed packet descriptors.
      void iso_ret_submit(uint32_t seqnum, uint32_t devid, uint32_t direction, uint32_t ep,
                          int actual_length, const uint8_t *in_data, int in_len,
                          const std::vector<uint8_t> &iso_desc_in, int npkts) {
        std::vector<uint8_t> v;
        put32(v, RET_SUBMIT); put32(v, seqnum); put32(v, devid);
        put32(v, direction);  put32(v, ep);
        put32(v, 0);                          // status
        put32(v, (uint32_t) actual_length);   // actual_length
        put32(v, 0);                          // start_frame
        put32(v, (uint32_t) npkts);           // number_of_packets
        put32(v, 0);                          // error_count
        for (int i = 0; i < 8; i++) v.push_back(0);
        if (direction == DIR_IN && in_len > 0)
          v.insert(v.end(), in_data, in_data + in_len);
        for (int i = 0; i < npkts; i++) {
          const uint8_t *d = iso_desc_in.data() + (size_t) i * 16;
          uint32_t off = rd32(d), len = rd32(d + 4);
          put32(v, off); put32(v, len); put32(v, len); put32(v, 0);  // actual=len, status=0
        }
        send_all(v.data(), (int) v.size());
      }

      void ret_unlink(uint32_t seqnum, uint32_t devid, int32_t status) {
        std::vector<uint8_t> v;
        put32(v, RET_UNLINK); put32(v, seqnum); put32(v, devid);
        put32(v, 0); put32(v, 0);
        put32(v, (uint32_t) status);
        for (int i = 0; i < 24; i++) v.push_back(0);
        send_all(v.data(), (int) v.size());
      }

      // EP0 control transfer.
      void handle_control(uint32_t seqnum, uint32_t devid, uint32_t direction,
                          const uint8_t *setup, const uint8_t *out_data, int out_len) {
        uint8_t bmRequestType = setup[0];
        uint8_t bRequest = setup[1];
        uint16_t wValue = le16(setup + 2);
        uint16_t wLength = le16(setup + 6);
        uint8_t rtype = (bmRequestType >> 5) & 0x3;  // 0=std 1=class 2=vendor

        if (rtype == 0) {                            // standard
          if (bRequest == 0x06) {                    // GET_DESCRIPTOR
            uint8_t dtype = wValue >> 8, dindex = wValue & 0xFF;
            const uint8_t *data = nullptr; int dlen = 0;
            static const uint8_t lang[] = {0x04, 0x03, 0x09, 0x04};
            static const uint8_t s_mfr[] = {0x3E, 0x03, 'S',0,'o',0,'n',0,'y',0,' ',0,'I',0,
              'n',0,'t',0,'e',0,'r',0,'a',0,'c',0,'t',0,'i',0,'v',0,'e',0,' ',0,'E',0,'n',0,
              't',0,'e',0,'r',0,'t',0,'a',0,'i',0,'n',0,'m',0,'e',0,'n',0,'t',0};
            static const uint8_t s_prod[] = {0x28, 0x03, 'W',0,'i',0,'r',0,'e',0,'l',0,'e',0,
              's',0,'s',0,' ',0,'C',0,'o',0,'n',0,'t',0,'r',0,'o',0,'l',0,'l',0,'e',0,'r',0};
            std::vector<uint8_t> serial_desc;
            if (dtype == 0x01) { data = DEVICE_DESC; dlen = sizeof(DEVICE_DESC); }
            else if (dtype == 0x02) { data = CONFIG_DESC; dlen = sizeof(CONFIG_DESC); }
            else if (dtype == 0x22) { data = HID_REPORT_DESC; dlen = sizeof(HID_REPORT_DESC); }
            else if (dtype == 0x21) { data = CONFIG_DESC + 204; dlen = 9; }  // HID desc @off 204
            else if (dtype == 0x03) {                // string
              if (dindex == 0) { data = lang; dlen = sizeof(lang); }
              else if (dindex == 1) { data = s_mfr; dlen = sizeof(s_mfr); }
              else if (dindex == 2) { data = s_prod; dlen = sizeof(s_prod); }
              else if (dindex == 3) {
                build_string_desc(slot ? slot->serial : std::string("00000000"), serial_desc);
                data = serial_desc.data(); dlen = (int) serial_desc.size();
              } else { ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0); return; }
            }
            int n = (dlen < (int) wLength) ? dlen : (int) wLength;
            ret_submit(seqnum, devid, direction, 0, 0, data, n);
            return;
          }
          ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);  // SET_CONFIG/IFACE/etc
          return;
        }
        if (rtype == 1) {                            // HID class
          if (bRequest == 0x01) {                    // GET_REPORT (feature)
            uint8_t rid = wValue & 0xFF;
            uint8_t buf[64] = {0};
            const uint8_t *f = nullptr;
            // Prefer a live feature report captured from the real controller.
            if (slot && slot->on_feature) {
              int got = slot->on_feature(rid, buf);
              if (got > 0) f = buf;
            }
            if (!f) {
              if (rid == 0x05) f = FEAT_05;
              else if (rid == 0x09) f = FEAT_09;
              else if (rid == 0x20) f = FEAT_20;
              else if (rid == 0x81) f = FEAT_81;
              else { buf[0] = rid; f = buf; }
            }
            int n = (64 < (int) wLength) ? 64 : (int) wLength;
            ret_submit(seqnum, devid, direction, 0, 0, f, n);
            return;
          }
          if (bRequest == 0x09) {                    // SET_REPORT (output via EP0)
            if (out_len > 1 && out_data[0] == 0x02 && slot && slot->on_output)
              slot->on_output(out_data + 1);
            ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);
            return;
          }
          ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);  // SET_IDLE / SET_PROTOCOL
          return;
        }
        ret_submit(seqnum, devid, direction, 0, 0, nullptr, 0);
      }

      // ~250 Hz interrupt-IN sender.
      void input_sender() {
        while (!stop.load()) {
          std::pair<uint32_t, uint32_t> job; bool have = false;
          {
            std::lock_guard<std::mutex> lk(pend_mtx);
            if (!pending_in.empty()) { job = pending_in.front(); pending_in.pop_front(); have = true; }
          }
          if (have) {
            uint8_t rep[INPUT_REPORT_LEN];
            { std::lock_guard<std::mutex> lk(slot->input_mtx);
              std::memcpy(rep, slot->input_report, INPUT_REPORT_LEN); }
            ret_submit(job.first, job.second, DIR_IN, 4, 0, rep, INPUT_REPORT_LEN);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
      }

      void run() {
        std::thread sender(&session_t::input_sender, this);
        uint8_t hdr[48];
        while (!stop.load()) {
          if (!recv_n(hdr, 48)) break;
          uint32_t command = rd32(hdr);
          uint32_t seqnum = rd32(hdr + 4), devid = rd32(hdr + 8);
          uint32_t direction = rd32(hdr + 12), ep = rd32(hdr + 16);
          if (command == CMD_SUBMIT) {
            uint32_t xfer_len = rd32(hdr + 24);
            int32_t  npkts = (int32_t) rd32(hdr + 32);   // -1 (0xFFFFFFFF) for non-iso
            const uint8_t *setup = hdr + 40;
            std::vector<uint8_t> out_data;
            if (direction == DIR_OUT && xfer_len > 0) {
              out_data.resize(xfer_len);
              if (!recv_n(out_data.data(), (int) xfer_len)) break;
            }
            bool is_iso = (npkts >= 0);
            std::vector<uint8_t> iso_desc;
            if (is_iso) {
              if (npkts > 1024) break;                 // sanity
              if (npkts > 0) {
                iso_desc.resize((size_t) npkts * 16);
                if (!recv_n(iso_desc.data(), (int) iso_desc.size())) break;
              }
            }
            if (ep == 0) {
              handle_control(seqnum, devid, direction, setup,
                             out_data.data(), (int) out_data.size());
            } else if (is_iso) {
              // Complete the iso URB so the virtual audio endpoint never stalls.
              // Phase 1 discards the PCM (no HD-haptic capture yet); Phase 2 hooks
              // the iso-OUT payload here to reconstruct the 0x36 haptic stream.
              std::vector<uint8_t> in_buf;
              int in_len = 0;
              if (direction == DIR_IN) { in_len = (int) xfer_len; in_buf.assign(in_len, 0); }
              iso_ret_submit(seqnum, devid, direction, ep,
                             direction == DIR_OUT ? (int) xfer_len : in_len,
                             in_buf.data(), in_len, iso_desc, npkts);
            } else if (direction == DIR_IN) {
              std::lock_guard<std::mutex> lk(pend_mtx);
              pending_in.emplace_back(seqnum, devid);
            } else {                                 // interrupt OUT — game output
              if (out_data.size() > 1 && out_data[0] == 0x02 && slot && slot->on_output)
                slot->on_output(out_data.data() + 1);
              ret_submit(seqnum, devid, direction, ep, 0, nullptr, 0);
            }
          } else if (command == CMD_UNLINK) {
            uint32_t unlink_seq = rd32(hdr + 20);
            { std::lock_guard<std::mutex> lk(pend_mtx);
              for (auto it = pending_in.begin(); it != pending_in.end();)
                it = (it->first == unlink_seq) ? pending_in.erase(it) : it + 1; }
            ret_unlink(seqnum, devid, 0);
          } else {
            break;
          }
        }
        stop.store(true);
        sender.join();
      }
    };

  }  // namespace

  // ===========================================================================
  //  usbip_ds5_device
  // ===========================================================================
  usbip_ds5_device::~usbip_ds5_device() { stop(); }

  std::string usbip_ds5_device::next_busid_locked() const {
    for (int i = 1;; ++i) {
      std::string cand = "1-" + std::to_string(i);
      bool used = false;
      for (auto &s : slots_) {
        if (s->busid == cand) { used = true; break; }
      }
      if (!used) return cand;
    }
  }

  std::shared_ptr<slot_t> usbip_ds5_device::add_slot(const std::string &serial,
                                                     slot_t::output_cb on_output,
                                                     slot_t::feature_cb on_feature,
                                                     std::string busid) {
    auto slot = std::make_shared<slot_t>();
    slot->serial = serial.empty() ? std::string("00000000") : serial;
    slot->on_output = std::move(on_output);
    slot->on_feature = std::move(on_feature);
    std::lock_guard<std::mutex> lk(slots_mtx_);
    slot->busid = busid.empty() ? next_busid_locked() : busid;
    slots_.push_back(slot);
    BOOST_LOG(info) << "ds5-bridge usbip: slot added busid="sv << slot->busid
                    << " serial="sv << slot->serial;
    return slot;
  }

  void usbip_ds5_device::remove_slot(const std::shared_ptr<slot_t> &slot) {
    if (!slot) return;
    std::lock_guard<std::mutex> lk(slots_mtx_);
    slots_.erase(std::remove(slots_.begin(), slots_.end(), slot), slots_.end());
    BOOST_LOG(info) << "ds5-bridge usbip: slot removed busid="sv << slot->busid;
  }

  void usbip_ds5_device::set_input(const std::shared_ptr<slot_t> &slot,
                                   const uint8_t report[INPUT_REPORT_LEN]) {
    if (!slot) return;
    std::lock_guard<std::mutex> lk(slot->input_mtx);
    std::memcpy(slot->input_report, report, INPUT_REPORT_LEN);
  }

  std::shared_ptr<slot_t> usbip_ds5_device::find_slot(const char *busid) {
    std::lock_guard<std::mutex> lk(slots_mtx_);
    for (auto &s : slots_) {
      if (std::strncmp(s->busid.c_str(), busid, s->busid.size()) == 0 &&
          busid[s->busid.size()] == '\0') {
        return s;
      }
    }
    return nullptr;
  }

  bool usbip_ds5_device::start() {
    if (running_.load()) return true;

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      BOOST_LOG(error) << "ds5-bridge usbip: WSAStartup failed"sv;
      return false;
    }
    SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) { BOOST_LOG(error) << "ds5-bridge usbip: socket() failed"sv; return false; }
    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(3240);
    InetPtonA(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (::bind(s, (sockaddr *) &addr, sizeof(addr)) != 0 || ::listen(s, 4) != 0) {
      BOOST_LOG(error) << "ds5-bridge usbip: bind/listen on 127.0.0.1:3240 failed (port in use?)"sv;
      ::closesocket(s);
      return false;
    }
    listen_sock_ = (uintptr_t) s;
    stop_.store(false);
    running_.store(true);
    accept_thread_ = std::thread(&usbip_ds5_device::accept_loop, this);
    BOOST_LOG(info) << "ds5-bridge usbip: virtual DualSense USB/IP server on 127.0.0.1:3240"sv;
    return true;
  }

  void usbip_ds5_device::accept_loop() {
    std::vector<std::thread> workers;
    while (!stop_.load()) {
      SOCKET cs = ::accept((SOCKET) listen_sock_, nullptr, nullptr);
      if (cs == INVALID_SOCKET) break;
      int one = 1;
      setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, (const char *) &one, sizeof(one));
      // Serve each vhci import on its own thread so multiple controllers can be
      // attached concurrently. Track the socket so stop() can unblock every
      // session parked in recv().
      { std::lock_guard<std::mutex> lk(client_socks_mtx_); client_socks_.push_back((uintptr_t) cs); }
      workers.emplace_back([this, cs] {
        serve_session((uintptr_t) cs);
        {
          std::lock_guard<std::mutex> lk(client_socks_mtx_);
          client_socks_.erase(std::remove(client_socks_.begin(), client_socks_.end(), (uintptr_t) cs),
                              client_socks_.end());
        }
        ::closesocket(cs);
      });
      // Reap finished workers opportunistically.
      workers.erase(std::remove_if(workers.begin(), workers.end(),
                                   [](std::thread &t) { return !t.joinable(); }),
                    workers.end());
    }
    for (auto &t : workers) {
      if (t.joinable()) t.join();
    }
  }

  void usbip_ds5_device::serve_session(uintptr_t client_sock) {
    SOCKET cs = (SOCKET) client_sock;
    uint8_t hdr[8];
    int off = 0;
    while (off < 8) {
      int r = ::recv(cs, (char *) hdr + off, 8 - off, 0);
      if (r <= 0) return;
      off += r;
    }
    uint16_t code = (uint16_t(hdr[2]) << 8) | hdr[3];

    if (code == OP_REQ_DEVLIST) {
      std::vector<std::shared_ptr<slot_t>> snapshot;
      { std::lock_guard<std::mutex> lk(slots_mtx_); snapshot = slots_; }
      std::vector<uint8_t> v;
      put16(v, USBIP_VERSION); put16(v, OP_REP_DEVLIST); put32(v, 0);
      put32(v, (uint32_t) snapshot.size());
      for (auto &sl : snapshot) append_usb_device(v, sl->busid, true);
      ::send(cs, (const char *) v.data(), (int) v.size(), 0);
      return;
    }
    if (code == OP_REQ_IMPORT) {
      uint8_t busid[32]; int boff = 0;
      while (boff < 32) { int r = ::recv(cs, (char *) busid + boff, 32 - boff, 0); if (r <= 0) return; boff += r; }
      busid[31] = '\0';
      auto slot = find_slot((const char *) busid);
      std::vector<uint8_t> v;
      put16(v, USBIP_VERSION); put16(v, OP_REP_IMPORT); put32(v, slot ? 0 : 1);
      if (slot) append_usb_device(v, slot->busid, false);
      ::send(cs, (const char *) v.data(), (int) v.size(), 0);
      if (!slot) return;

      BOOST_LOG(info) << "ds5-bridge usbip: vhci attached busid="sv << slot->busid;
      slot->attached.store(true);
      session_t sess;
      sess.sock = cs;
      sess.slot = slot;
      sess.run();
      slot->attached.store(false);
      BOOST_LOG(info) << "ds5-bridge usbip: vhci session ended busid="sv << slot->busid;
      return;
    }
  }

  void usbip_ds5_device::stop() {
    if (!running_.exchange(false)) return;
    stop_.store(true);
    if (listen_sock_ != ~uintptr_t(0)) {
      ::closesocket((SOCKET) listen_sock_);
      listen_sock_ = ~uintptr_t(0);
    }
    // Close every live vhci session socket so their worker threads unblock from
    // recv() and the accept loop can join them.
    {
      std::lock_guard<std::mutex> lk(client_socks_mtx_);
      for (uintptr_t s : client_socks_) ::closesocket((SOCKET) s);
    }
    if (accept_thread_.joinable()) accept_thread_.join();
    { std::lock_guard<std::mutex> lk(slots_mtx_); slots_.clear(); }
  }

}  // namespace platf::ds5_bridge
