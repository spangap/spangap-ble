/**
 * ble_sec — one security policy for the whole device, and the timed window that
 * decides when a new peer may join it.
 *
 * THE POLICY IS GLOBAL, THE DEMAND IS PER CHARACTERISTIC. There is one
 * `ble_hs_cfg`, so there is one io capability, one bonding stance and one bond
 * store. What differs between consumers is the ACCESS FLAGS on their own
 * characteristics: a service whose characteristics carry BLE_GATT_CHR_F_*_ENC
 * forces pairing before a read or a write lands, and a service whose do not
 * never triggers a pairing dialog at all. That split is what lets one device
 * demand a bond on one service and stay open on another.
 *
 * THE WINDOW. NimBLE has no hook that can refuse an incoming pairing request
 * before the exchange starts, so the window is enforced from both ends: while
 * it is closed `sm_bonding` is 0, so no long-term key is distributed and no
 * bond is written; and a peer that pairs anyway is unpaired and dropped the
 * moment the exchange completes. Existing bonds are untouched by any of
 * this and reconnect whenever they like.
 *
 * Pairing is Just Works (NO_INPUT_OUTPUT). The RNS RNode client requires only
 * that the device be BONDED, not that the bond was authenticated, so the timed
 * window is what stands in for a passkey.
 */
#include "ble_priv.h"

#include "host/ble_store.h"

#include <cstring>

static uint32_t s_pairUntilMs = 0;   /* millis() deadline; 0 = closed */

/* Peers that paired inside the window keep their bond; anyone else is undone in
 * bleSecOnPairingComplete. The flag is read by the SM at pairing time, so
 * flipping it is what actually stops a bond being written. */
static bool pairingOpen(void) {
    return s_pairUntilMs != 0 && (int32_t)(s_pairUntilMs - millis()) > 0;
}

void bleSecConfigure(void) {
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_sc             = 1;    /* LE Secure Connections */
    ble_hs_cfg.sm_mitm           = 0;    /* Just Works has no MITM protection */
    ble_hs_cfg.sm_bonding        = pairingOpen() ? 1 : 0;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    /* The bond store is finite; round-robin eviction is what keeps a full store
     * from refusing the pairing an operator just asked for. */
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
}

void blePairingWindow(uint32_t seconds) {
    s_pairUntilMs = seconds ? millis() + seconds * 1000 : 0;
    ble_hs_cfg.sm_bonding = pairingOpen() ? 1 : 0;
    if (seconds) info("pairing open for %us", (unsigned)seconds);
    else         info("pairing closed");
    bleSecPublish();
    bleWake();
}

uint32_t blePairingLeft(void) {
    if (!pairingOpen()) return 0;
    return (s_pairUntilMs - millis() + 999) / 1000;
}

/* Called from the ble task's loop; the window closes on its own. */
void blePairingTick(void) {
    if (s_pairUntilMs && !pairingOpen()) {
        s_pairUntilMs = 0;
        ble_hs_cfg.sm_bonding = 0;
        info("pairing window closed");
        bleSecPublish();
    }
}

void bleSecPublish(void) {
    char buf[48];
    uint32_t left = blePairingLeft();
    if (left) snprintf(buf, sizeof(buf), "open, %us left", (unsigned)left);
    else      safeStrncpy(buf, "closed", sizeof(buf));
    storageBegin();
    storageSet("ble.pairing_text", buf);
    storageSet("ble.pairing_open", left ? 1 : 0);
    storageEnd();
}

/* ─────────────── the bond store ─────────────── */

static int bondList(ble_addr_t* out, int max) {
    int n = 0;
    if (ble_store_util_bonded_peers(out, &n, max) != 0) return 0;
    return n;
}

int bleBondCount(void) {
    if (!s_hostUp) return 0;
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    return bondList(peers, CONFIG_BT_NIMBLE_MAX_BONDS);
}

bool bleIsBonded(const uint8_t addr[6], uint8_t addrType) {
    if (!s_hostUp) return false;
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = bondList(peers, CONFIG_BT_NIMBLE_MAX_BONDS);
    for (int i = 0; i < n; i++)
        if (peers[i].type == addrType && std::memcmp(peers[i].val, addr, 6) == 0) return true;
    return false;
}

bool bleBondExists(const uint8_t addr[6]) {
    if (!s_hostUp) return false;
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = bondList(peers, CONFIG_BT_NIMBLE_MAX_BONDS);
    for (int i = 0; i < n; i++)
        if (std::memcmp(peers[i].val, addr, 6) == 0) return true;
    return false;
}

/* The Now variants run the actual unpair. BLE TASK ONLY — that task owns
 * the bond store's persistence, so mutations stay serialized against the
 * snapshot ble_store.cpp writes; the public wrappers in ble.cpp post here. */
bool bleForgetNow(const uint8_t addr[6]) {
    if (!s_hostUp) return false;
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = bondList(peers, CONFIG_BT_NIMBLE_MAX_BONDS);
    bool any = false;
    for (int i = 0; i < n; i++) {
        if (std::memcmp(peers[i].val, addr, 6) != 0) continue;
        if (ble_gap_unpair(&peers[i]) == 0) any = true;
    }
    if (any) {
        char a[20]; bleFmtAddr(addr, a, sizeof(a));
        info("forgot %s", a);
        bleStoreDirty();
    }
    return any;
}

void bleForgetAllNow(void) {
    if (!s_hostUp) return;
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = bondList(peers, CONFIG_BT_NIMBLE_MAX_BONDS);
    for (int i = 0; i < n; i++) ble_gap_unpair(&peers[i]);
    info("forgot %d bond%s", n, n == 1 ? "" : "s");
    if (n) bleStoreDirty();
}

/* ─────────────── pairing outcome (ble task) ─────────────── */

void bleSecOnPairingComplete(uint16_t conn, int status) {
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn, &d) != 0) return;
    char a[20]; bleFmtAddr(d.peer_id_addr.val, a, sizeof(a));

    if (status != 0) {
        s_pairFails++;
        warn("pairing with %s failed (status %d)", a, status);
        return;
    }
    if (!d.sec_state.bonded) return;      /* encrypted only; nothing was stored */

    ble_conn_t* c = bleConnFind(conn);
    if (pairingOpen() || (c && c->wasBonded)) {
        info("bonded %s", a);
        bleSecPublish();
        return;
    }
    /* A bond written outside the window. Undo it and drop the peer, so the only
     * way onto this device's bond list stays the operator opening the window. */
    warn("%s bonded outside the pairing window — undone", a);
    ble_gap_unpair(&d.peer_id_addr);
    ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    s_pairFails++;
}
