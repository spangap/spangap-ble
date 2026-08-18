file(GLOB BLE_SRCS "src/*.cpp")

include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
# Conditional sources: esp-idf/conditional/<name>/ is compiled only when <name>
# is staged. Nothing here needs one yet; the wiring stays so adding an
# on-device pane later is a directory, not a CMake edit.
idf_component_register(
    SRCS
        ${BLE_SRCS}
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "include" ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        bt
        esp_event
)

# NimBLE's store/config PRIVATE header, for ble_store.cpp: it maps the RAM
# store's arrays correctly in both the static and the vars-pointer build, and
# it is deliberately not exported by the bt component. Layout drift in a new
# IDF is caught twice — at compile time when the header moves or renames, and
# at runtime by the sizeof stamps in /state/ble/bonds.bin.
idf_component_get_property(_bt_dir bt COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${_bt_dir}/host/nimble/nimble/nimble/host/store/config/src")
