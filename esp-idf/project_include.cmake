# NimBLE host-based privacy, on a target Espressif's Kconfig gates it off for.
# The BT_NIMBLE_HOST_BASED_PRIVACY option is ESP32-only — that chip's
# controller lacks link-layer privacy, so there the cfg header forces the
# feature on unconditionally — but the mechanism itself is controller-agnostic:
# the host derives each resolvable private address from the identity resolving
# key and sets it through the ordinary LE Set Random Address command. This
# straddle needs the HOST to own the address — rotation on its own cadence, and
# a current value it can read back (bleOwnAddr, the who-dials election) — which
# the S3's controller-side privacy engine cannot give: its RPAs are generated
# out of the host's sight, and only when the resolving list is non-empty at
# all. esp_nimble_cfg.h wraps every MYNEWT_VAL in #ifndef, so predefining the
# two values here wins over the Kconfig fallback for the whole build, the bt
# component included. Runs at project scope (project_include.cmake), so it is
# in force before any component compiles.
idf_build_set_property(COMPILE_DEFINITIONS "MYNEWT_VAL_BLE_HOST_BASED_PRIVACY=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "MYNEWT_VAL_BLE_HS_PVCY=1" APPEND)
