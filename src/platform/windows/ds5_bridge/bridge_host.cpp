/**
 * @file src/platform/windows/ds5_bridge/bridge_host.cpp
 * @brief Host-side CTM-Bridge server + per-controller data-plane session.
 *
 * License-clean interop implementation (see bridge_host.h). No CTM source used.
 */
// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <enet/enet.h>

#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/bridge_host.h"
#include "src/platform/windows/ds5_bridge/ctmb_protocol.h"
#include "src/platform/windows/ds5_bridge/ds5_haptics.h"
#include "src/platform/windows/ds5_bridge/ds5_reports.h"
#include "src/platform/windows/ds5_bridge/vhci_attach.h"

using namespace std::literals;

namespace platf::ds5_bridge {

  namespace {
    uint64_t now_us() {
      using namespace std::chrono;
      return (uint64_t) duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    void ensure_enet() {
      static std::once_flag once;
      std::call_once(once, [] {
        if (enet_initialize() != 0) {
          BOOST_LOG(error) << "ds5-bridge: enet_initialize failed"sv;
        }
      });
    }
  }  // namespace

  // ===========================================================================
  //  bridge_session — one plugged controller: data-plane transport + usbip slot
  // ===========================================================================
  class bridge_session {
  public:
    bridge_session(usbip_ds5_device *usbip, std::string ctmb_busid, int dport, bool haptics):
        usbip_(usbip), ctmb_busid_(std::move(ctmb_busid)), dport_(dport), haptics_(haptics) {}

    ~bridge_session() { stop(); }

    void start() {
      stop_.store(false);
      thread_ = std::thread(&bridge_session::run, this);
    }

    void stop() {
      if (stop_.exchange(true)) {
        if (thread_.joinable()) thread_.join();
        return;
      }
      if (thread_.joinable()) thread_.join();
    }

    /// The data port this session owns — the controller's stable identity across
    /// reconnects. Set once at construction, never mutated, so read lock-free.
    int dport() const { return dport_; }

    /// Re-label a live session to the busid the TV issued on this reconnect, so a
    /// later BRIDGE_STOP resolves. Called from the control thread; guarded because
    /// the session thread reads the label for logging (see label()).
    void relabel(const std::string &busid) {
      std::lock_guard<std::mutex> lk(label_mtx_);
      ctmb_busid_ = busid;
    }

  private:
    std::string label() {
      std::lock_guard<std::mutex> lk(label_mtx_);
      return ctmb_busid_;
    }

    // ---- outbound message send (transport-agnostic) -----------------------
    void send_msg(uint16_t type, uint32_t flags, uint32_t request_id,
                  const uint8_t *payload, uint32_t len) {
      std::vector<uint8_t> msg(sizeof(ctmb_header_t) + len);
      auto *h = reinterpret_cast<ctmb_header_t *>(msg.data());
      h->magic = CTMB_MAGIC;
      h->version = CTMB_VERSION;
      h->type = type;
      h->flags = flags;
      h->sequence = seq_++;
      h->timestamp_us = now_us();
      h->request_id = request_id;
      h->payload_len = len;
      if (len) std::memcpy(msg.data() + sizeof(ctmb_header_t), payload, len);

      if (transport_ == ENET && peer_) {
        ENetPacket *pkt = enet_packet_create(msg.data(), msg.size(), ENET_PACKET_FLAG_RELIABLE);
        if (pkt && enet_peer_send(peer_, 0, pkt) == 0) {
          enet_host_flush(host_);
        } else if (pkt) {
          enet_packet_destroy(pkt);
        }
      } else if (transport_ == TCP && tcp_client_ != INVALID_SOCKET) {
        int off = 0, total = (int) msg.size();
        while (off < total) {
          int n = ::send(tcp_client_, (const char *) msg.data() + off, total - off, 0);
          if (n <= 0) { link_down_ = true; break; }
          off += n;
        }
      }
    }

    // ---- inbound message dispatch -----------------------------------------
    void on_message(const ctmb_header_t *h, const uint8_t *payload) {
      switch (h->type) {
        case CTMB_MSG_HELLO:
          on_hello(h, payload);
          break;
        case CTMB_MSG_INPUT_REPORT:
          if (slot_) {
            uint8_t usb[INPUT_REPORT_LEN];
            if (bt_input_to_usb(payload, h->payload_len, usb)) {
              usbip_->set_input(slot_, usb);
            }
          }
          break;
        case CTMB_MSG_FEATURE_REPORT: {
          uint8_t rid = (uint8_t) h->request_id;
          if ((h->flags & CTMB_FLAG_OK) && h->payload_len > 0) {
            std::lock_guard<std::mutex> lk(feat_mtx_);
            auto &slot = feat_cache_[rid];
            slot.fill(0);
            std::memcpy(slot.data(), payload, std::min<uint32_t>(h->payload_len, 64));
            feat_have_[rid] = true;
          }
          break;
        }
        default:
          break;  // LOG/ERROR/ENUM/etc. — not used by the DS5 path
      }
    }

    void on_hello(const ctmb_header_t *h, const uint8_t *payload) {
      if (h->payload_len < sizeof(ctmb_device_caps_t)) return;
      ctmb_device_caps_t caps;
      std::memcpy(&caps, payload, sizeof(caps));
      caps.serial[sizeof(caps.serial) - 1] = '\0';
      std::string serial = caps.serial[0] ? caps.serial : label();

      // First HELLO on this session: register the virtual DualSense (its output +
      // feature callbacks run on a usbip server thread) and attach the vhci once.
      // A HELLO on a reconnect keeps the same slot + device (the game never sees a
      // hot-unplug); only the handshake below is replayed.
      if (!slot_) {
        slot_ = usbip_->add_slot(
          serial,
          [this](const uint8_t *eff) { on_game_output(eff); },
          [this](uint8_t rid, uint8_t *out) -> int {
            std::lock_guard<std::mutex> lk(feat_mtx_);
            if (feat_have_[rid]) {
              std::memcpy(out, feat_cache_[rid].data(), 64);
              return 64;
            }
            return 0;
          });
        // Phase 2: capture the game's iso-OUT PCM and drive a paced 0x36 haptic
        // stream. Wire the hook before the attach below (which is when the game
        // starts writing), and start the 10 ms pacer. Off unless configured.
        if (haptics_) {
          hap_ = std::make_unique<ds5_haptic_builder>();
          slot_->on_iso_out = [this](const uint8_t *pcm, size_t len) {
            if (hap_) hap_->feed_pcm(pcm, len);
          };
          pacer_stop_.store(false);
          pacer_thread_ = std::thread(&bridge_session::pacer_run, this);
        }
        // Attach off the session thread: `usbip attach` spawns a CLI + two port
        // snapshots (a couple of seconds); the loop must keep servicing ENet in
        // the meantime or the client link would go unpinged.
        std::string busid = slot_->busid;
        attach_thread_ = std::thread([this, busid] { vhci_port_.store(vhci_attach(busid)); });
        BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " up (serial="sv
                        << serial << ", usbip busid="sv << busid
                        << (haptics_ ? ", haptics on"sv : ""sv) << ")"sv;
      }

      // Always (re)answer the handshake: the client waits for HOST_CONFIG on every
      // (re)connect (needs_host_config), so this must be sent each HELLO.
      ctmb_host_config_t cfg {};
      cfg.bt_pace_us = 10667;
      cfg.input_report_len = caps.input_report_len;
      cfg.output_report_len = BT_OUTPUT_LEN;
      cfg.feature_report_len = caps.feature_report_len;
      cfg.paced_report_count = 1;
      cfg.paced_report_ids[0] = 0x36;
      send_msg(CTMB_MSG_HOST_CONFIG, CTMB_FLAG_OK, 0,
               reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg));

      // Prefetch calibration / MAC / firmware so the game's EP0 GET_REPORT is
      // answered from the real controller. This read also unlocks the pad's
      // extended (0x31) BT input mode, so refresh it on every (re)connect.
      for (uint8_t rid : {0x05, 0x09, 0x20}) {
        uint8_t req[64] = {0};
        req[0] = rid;
        send_msg(CTMB_MSG_FEATURE_GET, CTMB_FLAG_OK, rid, req, sizeof(req));
      }
    }

    // usbip server thread: the game wrote a 47-byte USB output effects block.
    void on_game_output(const uint8_t *eff) {
      uint8_t bt[BT_OUTPUT_LEN];
      usb_output_to_bt(eff, out_seq_++, bt);
      std::lock_guard<std::mutex> lk(out_mtx_);
      outbox_.emplace_back(bt, bt + BT_OUTPUT_LEN);
    }

    void drain_outbox() {
      std::deque<std::vector<uint8_t>> pending;
      { std::lock_guard<std::mutex> lk(out_mtx_); pending.swap(outbox_); }
      for (auto &bt : pending) {
        // 0x36 audio/haptic reports are paced (HOST_CONFIG advertises them); the
        // TV routes PACED output onto its 10.667 ms raw-ACL injector.
        uint32_t flags = CTMB_FLAG_OK;
        if (!bt.empty() && bt[0] == 0x36) flags |= CTMB_FLAG_PACED;
        send_msg(CTMB_MSG_OUTPUT_REPORT, flags, 0, bt.data(), (uint32_t) bt.size());
      }
    }

    // 10 ms grid producing paced 0x36 haptic reports from the game's iso-OUT
    // PCM. Enqueues to the outbox (drained on the run/session thread) so the
    // ENet host is only ever serviced from one thread.
    void pacer_run() {
      using namespace std::chrono;
      auto next = steady_clock::now();
      const auto period = microseconds(10000);  // 100 reports/s
      uint8_t rep[DS5_0X36_LEN];
      while (!pacer_stop_.load() && !stop_.load()) {
        next += period;
        std::this_thread::sleep_until(next);
        if (!hap_) continue;
        if (hap_->build_0x36(rep)) {
          std::lock_guard<std::mutex> lk(out_mtx_);
          if (outbox_.size() < 256) outbox_.emplace_back(rep, rep + DS5_0X36_LEN);
        }
      }
    }

    // ---- transport setup + main loop --------------------------------------
    void run() {
      ensure_enet();

      ENetAddress eaddr {};
      enet_address_set_host(&eaddr, "0.0.0.0");
      enet_address_set_port(&eaddr, (uint16_t) dport_);
      host_ = enet_host_create(AF_INET, &eaddr, 2, 2, 0, 0);
      if (!host_) {
        BOOST_LOG(warning) << "ds5-bridge: enet_host_create on port "sv << dport_
                           << " failed; ENet path unavailable"sv;
      }

      SOCKET ls = open_tcp_listener();
      u_long nb = 1;
      if (ls != INVALID_SOCKET) ioctlsocket(ls, FIONBIO, &nb);

      while (!stop_.load()) {
        // 1) Select a transport (ENet CONNECT wins if it arrives; else TCP accept).
        if (transport_ == NONE) {
          if (host_) {
            ENetEvent ev;
            if (enet_host_service(host_, &ev, 2) > 0 && ev.type == ENET_EVENT_TYPE_CONNECT) {
              peer_ = ev.peer;
              transport_ = ENET;
              BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " transport=ENet"sv;
            }
          }
          if (transport_ == NONE && ls != INVALID_SOCKET) {
            SOCKET c = ::accept(ls, nullptr, nullptr);
            if (c != INVALID_SOCKET) {
              int one = 1;
              setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char *) &one, sizeof(one));
              ioctlsocket(c, FIONBIO, &nb);
              tcp_client_ = c;
              transport_ = TCP;
              BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " transport=TCP"sv;
            }
          }
          if (transport_ == NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
          }
        }

        // 2) Pump inbound messages.
        if (transport_ == ENET) {
          pump_enet();
        } else {
          pump_tcp();
        }

        // 3) Flush outbound (paced BT output reports produced by the game).
        drain_outbox();

        // 4) A dropped link recycles the transport (the client reconnects to the
        // same port and re-HELLOs) — the virtual device + vhci stay attached.
        if (link_down_) reset_transport();
      }

      teardown(ls);
    }

    void reset_transport() {
      if (tcp_client_ != INVALID_SOCKET) { ::closesocket(tcp_client_); tcp_client_ = INVALID_SOCKET; }
      peer_ = nullptr;
      rxbuf_.clear();
      transport_ = NONE;
      link_down_ = false;
      BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " link dropped; awaiting reconnect"sv;
    }

    void pump_enet() {
      ENetEvent ev;
      int rc = enet_host_service(host_, &ev, 2);
      while (rc > 0) {
        if (ev.type == ENET_EVENT_TYPE_RECEIVE) {
          decode_stream(ev.packet->data, ev.packet->dataLength);
          enet_packet_destroy(ev.packet);
        } else if (ev.type == ENET_EVENT_TYPE_DISCONNECT) {
          link_down_ = true;
          break;
        }
        rc = enet_host_service(host_, &ev, 0);
      }
    }

    void pump_tcp() {
      uint8_t buf[2048];
      int n = ::recv(tcp_client_, (char *) buf, sizeof(buf), 0);
      if (n > 0) {
        rxbuf_.insert(rxbuf_.end(), buf, buf + n);
        drain_rxbuf();
      } else if (n == 0) {
        link_down_ = true;
      } else {
        int e = WSAGetLastError();
        if (e != WSAEWOULDBLOCK) link_down_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    // For ENet, one CTMB message per packet — but be tolerant and frame it too.
    void decode_stream(const uint8_t *data, size_t len) {
      rxbuf_.insert(rxbuf_.end(), data, data + len);
      drain_rxbuf();
    }

    void drain_rxbuf() {
      size_t off = 0;
      while (rxbuf_.size() - off >= sizeof(ctmb_header_t)) {
        ctmb_header_t h;
        std::memcpy(&h, rxbuf_.data() + off, sizeof(h));
        if (h.magic != CTMB_MAGIC || h.payload_len > CTMB_MAX_PAYLOAD) {
          // Desync — drop everything and resync on the next packet boundary.
          rxbuf_.clear();
          return;
        }
        if (rxbuf_.size() - off < sizeof(ctmb_header_t) + h.payload_len) break;
        const uint8_t *pl = rxbuf_.data() + off + sizeof(ctmb_header_t);
        on_message(&h, pl);
        off += sizeof(ctmb_header_t) + h.payload_len;
      }
      if (off > 0) rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + off);
    }

    SOCKET open_tcp_listener() {
      SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      if (s == INVALID_SOCKET) return INVALID_SOCKET;
      int yes = 1;
      setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
      sockaddr_in a {};
      a.sin_family = AF_INET;
      a.sin_addr.s_addr = INADDR_ANY;
      a.sin_port = htons((uint16_t) dport_);
      if (::bind(s, (sockaddr *) &a, sizeof(a)) != 0 || ::listen(s, 1) != 0) {
        ::closesocket(s);
        return INVALID_SOCKET;
      }
      u_long nb = 1;
      ioctlsocket(s, FIONBIO, &nb);
      return s;
    }

    void teardown(SOCKET ls) {
      pacer_stop_.store(true);
      if (pacer_thread_.joinable()) pacer_thread_.join();
      if (attach_thread_.joinable()) attach_thread_.join();
      if (slot_) {
        vhci_detach(vhci_port_.load());
        usbip_->remove_slot(slot_);
        slot_.reset();
      }
      if (tcp_client_ != INVALID_SOCKET) { ::closesocket(tcp_client_); tcp_client_ = INVALID_SOCKET; }
      if (ls != INVALID_SOCKET) ::closesocket(ls);
      if (host_) { enet_host_destroy(host_); host_ = nullptr; }
      peer_ = nullptr;
    }

    enum transport_e { NONE, ENET, TCP };

    usbip_ds5_device *usbip_;
    std::mutex label_mtx_;
    std::string ctmb_busid_;  // guarded by label_mtx_ (relabel from control thread)
    int dport_;
    bool haptics_ {false};

    std::unique_ptr<ds5_haptic_builder> hap_;   // Phase 2 0x36 builder (if haptics_)
    std::thread pacer_thread_;
    std::atomic<bool> pacer_stop_ {false};

    std::atomic<bool> stop_ {false};
    bool link_down_ {false};
    std::thread thread_;

    transport_e transport_ {NONE};
    ENetHost *host_ {nullptr};
    ENetPeer *peer_ {nullptr};
    SOCKET tcp_client_ {INVALID_SOCKET};
    std::vector<uint8_t> rxbuf_;

    std::shared_ptr<slot_t> slot_;
    std::thread attach_thread_;
    std::atomic<int> vhci_port_ {-1};
    uint32_t seq_ {0};
    uint8_t out_seq_ {0};

    std::mutex out_mtx_;
    std::deque<std::vector<uint8_t>> outbox_;

    std::mutex feat_mtx_;
    std::array<std::array<uint8_t, 64>, 256> feat_cache_ {};
    std::array<bool, 256> feat_have_ {};
  };

  // ===========================================================================
  //  bridge_host
  // ===========================================================================
  bridge_host::bridge_host() = default;
  bridge_host::~bridge_host() { stop(); }

  bool bridge_host::start(int port) {
    if (running_.load()) return true;
    port_ = port > 0 ? port : 48054;

    WSADATA wsa;
    static std::once_flag wsa_once;
    std::call_once(wsa_once, [&] { WSAStartup(MAKEWORD(2, 2), &wsa); });

    if (!usbip_.start()) {
      BOOST_LOG(error) << "ds5-bridge: usbip device failed to start; bridge disabled"sv;
      return false;
    }

    // Control-plane TCP listener (BRIDGE_START/STOP) + UDP discovery responder.
    SOCKET ts = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    SOCKET us = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(ts, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    setsockopt(us, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    sockaddr_in a {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons((uint16_t) port_);
    if (ts == INVALID_SOCKET || us == INVALID_SOCKET ||
        ::bind(ts, (sockaddr *) &a, sizeof(a)) != 0 || ::listen(ts, 4) != 0 ||
        ::bind(us, (sockaddr *) &a, sizeof(a)) != 0) {
      BOOST_LOG(error) << "ds5-bridge: control plane bind on port "sv << port_
                       << " failed (in use?); bridge disabled"sv;
      if (ts != INVALID_SOCKET) ::closesocket(ts);
      if (us != INVALID_SOCKET) ::closesocket(us);
      usbip_.stop();
      return false;
    }
    tcp_listen_ = (uintptr_t) ts;
    udp_sock_ = (uintptr_t) us;
    stop_.store(false);
    running_.store(true);
    control_thread_ = std::thread(&bridge_host::control_loop, this);
    BOOST_LOG(info) << "ds5-bridge: native provider up (control plane :"sv << port_
                    << ", usbip 127.0.0.1:3240)"sv;
    return true;
  }

  void bridge_host::control_loop() {
    SOCKET ts = (SOCKET) tcp_listen_;
    SOCKET us = (SOCKET) udp_sock_;
    while (!stop_.load()) {
      fd_set rd;
      FD_ZERO(&rd);
      FD_SET(ts, &rd);
      FD_SET(us, &rd);
      timeval tv { 0, 200000 };  // 200 ms so stop_ is checked promptly
      int r = ::select(0, &rd, nullptr, nullptr, &tv);
      if (r <= 0) continue;

      if (FD_ISSET(us, &rd)) {
        char buf[256];
        sockaddr_in from {};
        int flen = sizeof(from);
        int n = ::recvfrom(us, buf, sizeof(buf) - 1, 0, (sockaddr *) &from, &flen);
        if (n > 0) {
          buf[n] = '\0';
          if (std::strncmp(buf, "CTM_DISCOVER_V1", 15) == 0) {
            std::string reply = "CTM_AGENT_V1 port=" + std::to_string(port_);
            ::sendto(us, reply.c_str(), (int) reply.size(), 0, (sockaddr *) &from, flen);
          }
        }
      }

      if (FD_ISSET(ts, &rd)) {
        SOCKET c = ::accept(ts, nullptr, nullptr);
        if (c == INVALID_SOCKET) continue;
        // One command per connection (matches the client's connect/send/recv/close).
        std::string line;
        char ch;
        int guard = 0;
        while (guard++ < 512) {
          int k = ::recv(c, &ch, 1, 0);
          if (k <= 0) break;
          if (ch == '\n') break;
          line.push_back(ch);
        }
        std::string resp = handle_command(line);
        resp.push_back('\n');
        ::send(c, resp.c_str(), (int) resp.size(), 0);
        ::closesocket(c);
      }
    }
  }

  std::string bridge_host::handle_command(const std::string &line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd == "BRIDGE_START") {
      std::string kind, busid;
      int dport = 0;
      iss >> kind >> dport >> busid;
      if (kind != "ds5") {
        return "ERR unsupported kind (native provider is DS5-only in this build)";
      }
      if (dport <= 0 || busid.empty()) {
        return "ERR bad args";
      }
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      // The data port is a controller's stable identity across reconnects: the TV
      // reuses it but issues a fresh busid each time (ctm-ds5-1 -> ctm-ds5-2 ...).
      // If a session already owns this dport, adopt it — its ENet host, usbip slot
      // and vhci attach stay up, so the game never sees a re-plug — and just
      // relabel it to the incoming busid so a later BRIDGE_STOP resolves. Spinning
      // up a second session would only race for the same port: enet_host_create
      // fails and the loser lingers as a zombie TCP-fallback session the
      // reconnecting peer never reaches.
      for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        if (it->second->dport() != dport) continue;
        if (it->first != busid) {
          auto sess = std::move(it->second);
          sessions_.erase(it);
          sess->relabel(busid);
          sessions_[busid] = std::move(sess);
        }
        BOOST_LOG(info) << "ds5-bridge: BRIDGE_START ds5 port="sv << dport << " busid="sv
                        << busid << " (reconnect; adopted live session on this port)"sv;
        return "OK";
      }
      auto sess = std::make_unique<bridge_session>(&usbip_, busid, dport, haptics_.load());
      sess->start();
      sessions_[busid] = std::move(sess);
      BOOST_LOG(info) << "ds5-bridge: BRIDGE_START ds5 port="sv << dport << " busid="sv << busid;
      return "OK";
    }
    if (cmd == "BRIDGE_STOP") {
      std::string busid;
      iss >> busid;
      stop_session(busid);
      return "OK";
    }
    return "ERR unknown command";
  }

  void bridge_host::stop_session(const std::string &busid) {
    std::unique_ptr<bridge_session> victim;
    {
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      auto it = sessions_.find(busid);
      if (it == sessions_.end()) return;
      victim = std::move(it->second);
      sessions_.erase(it);
    }
    if (victim) victim->stop();  // join outside the lock
    BOOST_LOG(info) << "ds5-bridge: BRIDGE_STOP busid="sv << busid;
  }

  void bridge_host::stop() {
    if (!running_.exchange(false)) return;
    stop_.store(true);
    if (tcp_listen_ != ~uintptr_t(0)) { ::closesocket((SOCKET) tcp_listen_); tcp_listen_ = ~uintptr_t(0); }
    if (udp_sock_ != ~uintptr_t(0)) { ::closesocket((SOCKET) udp_sock_); udp_sock_ = ~uintptr_t(0); }
    if (control_thread_.joinable()) control_thread_.join();
    {
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      for (auto &kv : sessions_) kv.second->stop();
      sessions_.clear();
    }
    usbip_.stop();
    BOOST_LOG(info) << "ds5-bridge: native provider stopped"sv;
  }

}  // namespace platf::ds5_bridge
