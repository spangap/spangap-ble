/**
 * ble_gap — the radio's two shared instruments: the single legacy advertising
 * instance and the single scanner, plus the one GAP event handler every
 * connection this straddle owns is opened with.
 *
 * ADVERTISING IS ROUND-ROBINED. There is one legacy instance and one 31-byte
 * payload, and a 128-bit service UUID plus a name does not fit in it — so a
 * slot advertises alone for BLE_ADV_SLOT_MS with its UUID in the advertisement
 * and its name in the scan response, then the next slot takes over. Extended
 * advertising would carry an instance per consumer, but Android scanners use
 * legacy scan parameters by default and would not see an extended-only
 * advertisement at all.
 *
 * A central dials an ADDRESS, not a payload. A peer that already knows this
 * device therefore connects during whichever slot happens to be up, as long as
 * that slot is connectable — only DISCOVERY of a not-currently-advertised
 * payload waits for the rotation.
 *
 * HOST CONTEXT. bleGapEvent runs on the NimBLE host task and does one thing:
 * it copies what it needs into a queue slot and wakes the ble task. Every
 * decision — the connection table, the budget, the fan-out to consumers — is
 * made in bleGapDrain(), on the ble task.
 */
#include "ble_priv.h"

#include "freertos/queue.h"

#include <cstring>

/* One slot's turn at the advertisement. Long enough that the rotation costs
 * little radio time, short enough that a scanner running Columba's 5 s
 * discovery interval sees every payload within one of its passes. */
#define BLE_ADV_SLOT_MS      2000
#define BLE_ADV_ITVL_MS      100      /* advertising interval within a slot */

#define BLE_SCAN_ITVL_MS_DEF 100
#define BLE_SCAN_WIN_MS_DEF   30

#define BLE_HEV_QDEPTH        16

/* The largest inbound notification handed to a consumer. 512 is the ceiling an
 * ATT value can reach at the preferred MTU of 517. */
#define BLE_NOTIFY_RX_MAX    512

/* ─────────────── host → ble task ─────────────── */

static QueueHandle_t s_hev = nullptr;

static void bleNotifyArmInit(void);

bool bleGapQueueInit(void) {
    if (!s_hev) s_hev = xQueueCreate(BLE_HEV_QDEPTH, sizeof(ble_event_t));
    bleNotifyArmInit();
    return s_hev != nullptr;
}

static void hevPost(const ble_event_t* ev) {
    if (!s_hev) return;
    if (xQueueSend(s_hev, ev, 0) != pdTRUE) {
        /* Full — usually a scan-report burst. The ble task reports the count;
         * a lost DIAL outcome is recovered by the dial watchdog. */
        s_hevDrops++;
        return;
    }
    bleWake();
}

/* ─────────────── notification flow control ───────────────
 *
 * A conn handle parked here by bleNotifyArm() until one of its notifications
 * completes. Aligned 16-bit slots written by one task and read by the host
 * task — a torn read is not possible and a stale one only costs a wake. */
static volatile uint16_t s_notifyArm[BLE_MAX_CONNS];

static void bleNotifyArmInit(void) {
    for (auto& a : s_notifyArm) a = BLE_HS_CONN_HANDLE_NONE;
}

/* The one consumer callback that runs in host context. */
static ble_notify_rx_cb_t s_notifyRx = nullptr;

void bleOnNotifyRx(ble_notify_rx_cb_t cb) { s_notifyRx = cb; }

void bleNotifyArm(uint16_t conn) {
    for (auto& a : s_notifyArm) if (a == conn) return;          /* already armed */
    for (auto& a : s_notifyArm) if (a == BLE_HS_CONN_HANDLE_NONE) { a = conn; return; }
    warn("notify arm table full");
}

/* ─────────────── advertising ─────────────── */

static bool     s_advOn      = false;
static int      s_advSlot    = -1;      /* the slot currently on air */
static TickType_t s_advSince = 0;

void bleAdvStop(void) {
    if (!s_advOn) return;
    ble_gap_adv_stop();
    s_advOn   = false;
    s_advSlot = -1;
}

void bleGapOnHostDown(void) {
    s_advOn   = false;
    s_advSlot = -1;
}

/* Next used slot after `from`, wrapping; -1 when nobody is advertising. */
static int advNextSlot(int from) {
    for (int i = 1; i <= BLE_MAX_ADV_SLOTS; i++) {
        int k = (from + i) % BLE_MAX_ADV_SLOTS;
        if (s_adv[k].used) return k;
    }
    return -1;
}

static int advUsedCount(void) {
    int n = 0;
    for (auto& a : s_adv) if (a.used) n++;
    return n;
}

static bool advStartSlot(int slot) {
    ble_adv_req_t req;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_adv[slot].used) { xSemaphoreGive(s_lock); return false; }
    req = s_adv[slot].req;
    xSemaphoreGive(s_lock);

    ble_gap_adv_stop();          /* legacy data changes are stop → set → start */
    /* Off as of now; only a successful start below turns it back on, so a
     * failure leaves !s_advOn and the next reconcile pass retries. */
    s_advOn   = false;
    s_advSlot = -1;

    struct ble_hs_adv_fields adv = {};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    ble_uuid128_t svc;
    bool haveUuid = false;
    for (int i = 0; i < 16; i++) if (req.uuid128[i]) { haveUuid = true; break; }
    if (haveUuid) {
        svc.u.type = BLE_UUID_TYPE_128;
        std::memcpy(svc.value, req.uuid128, 16);
        adv.uuids128            = &svc;
        adv.num_uuids128        = 1;
        adv.uuids128_is_complete = 1;
    }
    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) { warn("adv_set_fields: rc=%d", rc); return false; }

    /* The name goes in the scan response: 31 bytes will not hold a 128-bit
     * UUID and a name together, and the UUID is what a scanner filters on. */
    struct ble_hs_adv_fields rsp = {};
    if (req.includeName && req.name[0]) {
        rsp.name             = (const uint8_t*)req.name;
        rsp.name_len         = (uint8_t)strnlen(req.name, BLE_ADV_NAME_MAX);
        rsp.name_is_complete = 1;
        rc = ble_gap_adv_rsp_set_fields(&rsp);
        if (rc != 0) warn("adv_rsp_set_fields: rc=%d", rc);
    }

    struct ble_gap_adv_params p = {};
    p.conn_mode = req.connectable ? BLE_GAP_CONN_MODE_UND : BLE_GAP_CONN_MODE_NON;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min  = (uint16_t)(BLE_ADV_ITVL_MS / 0.625);
    p.itvl_max  = (uint16_t)(BLE_ADV_ITVL_MS / 0.625);
    rc = ble_gap_adv_start(s_ownAddrType, nullptr, BLE_HS_FOREVER, &p,
                           bleGapEvent, nullptr);
    if (rc != 0) { warn("adv_start: rc=%d", rc); return false; }

    s_advOn    = true;
    s_advSlot  = slot;
    s_advSince = xTaskGetTickCount();
    s_advCycles++;
    return true;
}

void bleAdvReconcile(void) {
    int used = advUsedCount();
    if (used == 0) {
        if (s_advOn) { bleAdvStop(); info("advertising stopped (no claims)"); }
        s_advDirty = false;
        return;
    }

    bool rotate = s_advDirty
               || !s_advOn
               || s_advSlot < 0
               || !s_adv[s_advSlot].used
               || (used > 1 && (TickType_t)(xTaskGetTickCount() - s_advSince)
                                >= pdMS_TO_TICKS(BLE_ADV_SLOT_MS));
    if (!rotate) return;
    s_advDirty = false;

    /* A single claim never rotates — it is simply left on air, which is what
     * makes the round-robin free in the common case. */
    int next = (used == 1 && s_advSlot >= 0 && s_adv[s_advSlot].used)
                 ? s_advSlot : advNextSlot(s_advSlot);
    if (next < 0) { bleAdvStop(); return; }
    advStartSlot(next);
}

uint32_t bleAdvNextDueMs(void) {
    if (!s_advOn || advUsedCount() <= 1) return 1000;
    uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - s_advSince) * portTICK_PERIOD_MS);
    return elapsed >= BLE_ADV_SLOT_MS ? 0 : (BLE_ADV_SLOT_MS - elapsed);
}

/* ─────────────── scanning ─────────────── */

static bool s_scanOn = false;

/* The filter the DISC handler reads, in HOST context. A snapshot taken on the
 * ble task between disc_cancel and ble_gap_disc — reading s_scanReq there
 * would race a caller's re-parameterisation mid-write. */
static ble_scan_req_t s_scanLive = {};

void bleScanStopNow(void) {
    if (!s_scanOn) return;
    ble_gap_disc_cancel();
    s_scanOn = false;
}

void bleScanReconcile(void) {
    /* The controller cannot scan while it is connecting; the dirty flag
     * stays set and the outcome of the dial brings the scan back. */
    if (s_dialInFlight) return;
    if (!s_scanDirty && s_scanOn == s_scanWanted) return;
    s_scanDirty = false;

    if (!s_scanWanted) { bleScanStopNow(); return; }
    bleScanStopNow();

    ble_scan_req_t r;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    r = s_scanReq;
    xSemaphoreGive(s_lock);
    s_scanLive = r;

    uint16_t itvl = r.intervalMs ? r.intervalMs : BLE_SCAN_ITVL_MS_DEF;
    uint16_t win  = r.windowMs   ? r.windowMs   : BLE_SCAN_WIN_MS_DEF;
    /* Scanning is the expensive half of sharing 2.4 GHz with Wi-Fi and ESP-NOW,
     * and it is the half we can cheapen: halve the window while a connection is
     * attached, so an established link keeps its airtime. */
    if (bleConnCount() > 0) win = (uint16_t)(win / 2 ? win / 2 : 1);
    if (win > itvl) win = itvl;

    struct ble_gap_disc_params p = {};
    p.itvl              = (uint16_t)(itvl / 0.625);
    p.window            = (uint16_t)(win  / 0.625);
    p.passive           = r.activeScan ? 0 : 1;
    p.filter_duplicates = 0;    /* an address that rotates must be re-reported */
    int rc = ble_gap_disc(s_ownAddrType, BLE_HS_FOREVER, &p, bleGapEvent, nullptr);
    if (rc != 0) { warn("disc: rc=%d", rc); return; }
    s_scanOn = true;
}

/* ─────────────── advertisement parsing (host context) ─────────────── */

/* ESP-IDF hands over the raw advertisement bytes — there is no BlueZ-style
 * service filter — so the 128-bit UUID match is done here, once, rather than by
 * every consumer. */
static bool advMatches(const struct ble_hs_adv_fields* f, const uint8_t want[16],
                       uint8_t outUuid[16]) {
    bool wanted = false;
    for (int i = 0; i < 16; i++) if (want[i]) { wanted = true; break; }
    for (int i = 0; i < f->num_uuids128; i++) {
        if (!wanted) { std::memcpy(outUuid, f->uuids128[i].value, 16); return true; }
        if (std::memcmp(f->uuids128[i].value, want, 16) == 0) {
            std::memcpy(outUuid, want, 16);
            return true;
        }
    }
    if (!wanted) { std::memset(outUuid, 0, 16); return true; }
    return false;
}

/* ─────────────── what the scan has seen ───────────────
 *
 * Kept here, on the ble task, and fed by every scan report whoever asked for
 * the scan — so `ble scan` is a readout of the same instrument a consumer is
 * already using, rather than a second scanner competing with it. Bounded and
 * keyed by address; a rotating address simply lands as a new row and ages out.
 */
static ble_seen_t s_seen[BLE_MAX_SEEN] = {};

static void seenRecord(const ble_event_t* ev) {
    ble_seen_t* slot   = nullptr;
    ble_seen_t* free_  = nullptr;
    ble_seen_t* oldest = nullptr;
    for (auto& s : s_seen) {
        if (s.used && std::memcmp(s.addr, ev->addr, 6) == 0) { slot = &s; break; }
        if (!s.used) { if (!free_) free_ = &s; continue; }
        if (!oldest || (int32_t)(s.lastMs - oldest->lastMs) < 0) oldest = &s;
    }
    if (!slot) slot = free_ ? free_ : oldest;
    if (!slot) return;
    slot->used     = true;
    slot->rssi     = ev->rssi;
    slot->addrType = ev->addrType;
    slot->lastMs   = millis();
    std::memcpy(slot->addr, ev->addr, 6);
    std::memcpy(slot->uuid128, ev->uuid128, 16);
    if (ev->name[0]) safeStrncpy(slot->name, ev->name, sizeof(slot->name));
}

int bleSeenList(ble_seen_t* out, int max) {
    int n = 0;
    for (auto& s : s_seen) {
        if (!s.used || n >= max) continue;
        /* Older than a minute is not "what is out there" any more. */
        if (millis() - s.lastMs > 60000) { s.used = false; continue; }
        out[n++] = s;
    }
    return n;
}

void bleSeenClear(void) { std::memset(s_seen, 0, sizeof(s_seen)); }

/* ─────────────── the GAP event handler (host context) ─────────────── */

int bleGapEvent(struct ble_gap_event* event, void* /*arg*/) {
    ble_event_t ev = {};

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            /* A failed dial. The event is ZEROED on this path — conn_handle 0
             * is a live connection's handle, so it must not be read from here;
             * the ble task knows which address it was dialling. */
            ev.event  = BLE_IEV_DIAL_FAILED;
            ev.reason = (uint8_t)event->connect.status;
            hevPost(&ev);
            break;
        }
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(event->connect.conn_handle, &d) != 0) {
            /* The connection died in the race between the event and this
             * lookup. If a dial is pending this WAS its outcome — swallowing
             * it would leave s_dialInFlight set forever, which parks the
             * scan, every future dial and the address rotation. */
            if (s_dialInFlight) {
                ev.event  = BLE_IEV_DIAL_FAILED;
                ev.reason = 0;
                hevPost(&ev);
            }
            break;
        }
        ev.event    = BLE_EV_CONNECT;
        ev.conn     = event->connect.conn_handle;
        ev.central  = (d.role == BLE_GAP_ROLE_MASTER) ? 1 : 0;
        ev.addrType = d.peer_id_addr.type;
        ev.mtu      = ble_att_mtu(event->connect.conn_handle);
        ev.notify   = d.sec_state.bonded ? 1 : 0;   /* bonded BEFORE this session */
        std::memcpy(ev.addr, d.peer_id_addr.val, 6);
        hevPost(&ev);
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ev.event    = BLE_EV_DISCONNECT;
        ev.conn     = event->disconnect.conn.conn_handle;
        ev.reason   = (uint8_t)event->disconnect.reason;
        ev.addrType = event->disconnect.conn.peer_id_addr.type;
        std::memcpy(ev.addr, event->disconnect.conn.peer_id_addr.val, 6);
        hevPost(&ev);
        break;

    case BLE_GAP_EVENT_MTU:
        ev.event = BLE_EV_MTU;
        ev.conn  = event->mtu.conn_handle;
        ev.mtu   = event->mtu.value;
        hevPost(&ev);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ev.event  = BLE_EV_SUBSCRIBE;
        ev.conn   = event->subscribe.conn_handle;
        ev.attr   = event->subscribe.attr_handle;
        ev.notify = event->subscribe.cur_notify ? 1 : 0;
        hevPost(&ev);
        break;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Straight through to the consumer, in host context, because the
         * payload is far larger than an event message and copying it twice to
         * reach the same place would buy nothing. The handler's contract is
         * copy-and-post; see ble.h. */
        if (!s_notifyRx) break;
        uint8_t  buf[BLE_NOTIFY_RX_MAX];
        uint16_t n = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &n) != 0) break;
        s_notifyRx(event->notify_rx.conn_handle, event->notify_rx.attr_handle, buf, n);
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_TX: {
        /* One-shot, and only for a consumer that asked: this fires once per
         * notification, and an unconditional fan-out would put an ITS message
         * beside every chunk of a busy stream. */
        for (auto& armed : s_notifyArm) {
            if (armed != event->notify_tx.conn_handle) continue;
            armed    = BLE_HS_CONN_HANDLE_NONE;
            ev.event = BLE_EV_NOTIFY_TX;
            ev.conn  = event->notify_tx.conn_handle;
            ev.attr  = event->notify_tx.attr_handle;
            hevPost(&ev);
            break;
        }
        break;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ev.event = BLE_IEV_ADV_DONE;
        hevPost(&ev);
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
    case BLE_GAP_EVENT_PARING_COMPLETE:
        ev.event = BLE_IEV_PAIRED;
        ev.conn  = (event->type == BLE_GAP_EVENT_ENC_CHANGE)
                     ? event->enc_change.conn_handle
                     : event->pairing_complete.conn_handle;
        /* The status is a BLE_HS_* code, wider than `reason`; `attr` carries it
         * as an int16 for this internal event only. */
        ev.attr  = (uint16_t)(int16_t)((event->type == BLE_GAP_EVENT_ENC_CHANGE)
                     ? event->enc_change.status
                     : event->pairing_complete.status);
        hevPost(&ev);
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* The peer has a bond we no longer agree with. Drop ours and let it
         * pair again — the alternative is a device that can never recover from
         * a one-sided forget without the operator finding the menu. */
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &d) == 0)
            ble_gap_unpair(&d.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields f;
        if (ble_hs_adv_parse_fields(&f, event->disc.data, event->disc.length_data) != 0)
            break;
        uint8_t want[16];
        int8_t  floorRssi;
        std::memcpy(want, s_scanLive.uuid128, 16);
        floorRssi = s_scanLive.minRssi;
        if (floorRssi && event->disc.rssi < floorRssi) break;
        if (!advMatches(&f, want, ev.uuid128)) break;
        ev.event    = BLE_EV_SCAN;
        ev.addrType = event->disc.addr.type;
        ev.rssi     = event->disc.rssi;
        std::memcpy(ev.addr, event->disc.addr.val, 6);
        if (f.name && f.name_len) {
            size_t n = f.name_len < sizeof(ev.name) - 1 ? f.name_len : sizeof(ev.name) - 1;
            std::memcpy(ev.name, f.name, n);
            ev.name[n] = '\0';
        }
        hevPost(&ev);
        break;
    }

    default:
        break;
    }
    return 0;
}

/* ─────────────── drain (ble task) ─────────────── */

void bleGapDrain(void) {
    if (!s_hev) return;
    ble_event_t ev;
    while (xQueueReceive(s_hev, &ev, 0) == pdTRUE) {
        switch (ev.event) {

        case BLE_EV_CONNECT: {
            /* The table is exactly CONFIG_BT_NIMBLE_MAX_CONNECTIONS deep, so a
             * full table is the controller's own ceiling reached. Reservations
             * are not consulted here: an inbound connection carries no owner,
             * so there is nothing to charge it to — the budget is agreed at
             * bleSlotReserve() time and kept by each consumer capping its own
             * peers. */
            if (ev.central) {
                s_dialInFlight = false;   /* the dial's outcome, whatever the
                                           * table says below */
                /* The ATT MTU exchange is the central's to start — a
                 * peripheral cannot initiate it, and NimBLE does not do it on
                 * its own. Without it a dialled link sits at MTU 23 and a
                 * peer sizing its notifications to a bigger MTU loses bytes
                 * to the stack. The result arrives as BLE_EV_MTU. */
                int rc = ble_gattc_exchange_mtu(ev.conn, nullptr, nullptr);
                if (rc != 0) warn("mtu exchange: rc=%d", rc);
            }
            ble_conn_t* c = nullptr;
            for (auto& k : s_conns) if (!k.used) { c = &k; break; }
            if (!c) {
                char a[20]; bleFmtAddr(ev.addr, a, sizeof(a));
                warn("%s refused — all %d connections in use", a, BLE_MAX_CONNS);
                ble_gap_terminate(ev.conn, BLE_ERR_REM_USER_CONN_TERM);
                break;
            }
            c->used      = true;
            c->conn      = ev.conn;
            c->central   = ev.central;
            c->addrType  = ev.addrType;
            c->wasBonded = ev.notify;
            c->mtu       = ev.mtu ? ev.mtu : 23;
            std::memcpy(c->addr, ev.addr, 6);
            s_connTotal++;
            char a[20]; bleFmtAddr(c->addr, a, sizeof(a));
            info("connected %s (%s%s)", a, c->central ? "central" : "peripheral",
                 c->wasBonded ? ", bonded" : "");
            ev.notify = 0;                     /* the field means nothing upward */
            bleFire(&ev);
            s_scanDirty = true;                /* re-parameterise the duty cycle */
            break;
        }

        case BLE_EV_DISCONNECT: {
            ble_conn_t* c = bleConnFind(ev.conn);
            if (c) {
                char a[20]; bleFmtAddr(c->addr, a, sizeof(a));
                info("disconnected %s (reason 0x%02x)", a, (unsigned)ev.reason);
                if (!ev.addr[0] && !ev.addr[1]) std::memcpy(ev.addr, c->addr, 6);
                *c = ble_conn_t{};
            }
            for (auto& a : s_notifyArm) if (a == ev.conn) a = BLE_HS_CONN_HANDLE_NONE;
            bleFire(&ev);
            s_advDirty = true;   /* the slot is free again */
            s_scanDirty = true;
            /* A session ended: present a fresh address before the peer's next
             * attempt, so its per-address bookkeeping starts a clean file. */
            bleAddrRotateDue();
            break;
        }

        case BLE_EV_MTU: {
            ble_conn_t* c = bleConnFind(ev.conn);
            if (c) c->mtu = ev.mtu;
            bleFire(&ev);
            break;
        }

        case BLE_EV_SUBSCRIBE: {
            ble_conn_t* c = bleConnFind(ev.conn);
            if (c) {
                ev.mtu      = c->mtu;
                ev.addrType = c->addrType;
                std::memcpy(ev.addr, c->addr, 6);
            }
            bleStoreDirty();     /* a CCCD is a store record for a bonded peer */
            bleFire(&ev);
            break;
        }

        case BLE_EV_SCAN:
            s_scanSeen++;
            seenRecord(&ev);
            bleFire(&ev);
            break;

        case BLE_EV_NOTIFY_TX:
            bleFire(&ev);
            break;

        case BLE_IEV_PAIRED:
            bleSecOnPairingComplete(ev.conn, (int)(int16_t)ev.attr);
            bleStoreDirty();     /* a bond may have been written (or undone) */
            break;

        case BLE_IEV_ADV_DONE:
            s_advDirty = true;
            break;

        case BLE_IEV_DIAL_FAILED:
            s_dialInFlight = false;
            bleDialFailed(ev.reason);
            break;

        default:
            break;
        }
    }
}
