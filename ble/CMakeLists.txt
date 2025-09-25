cmake_minimum_required(VERSION 3.12)
set(PICO_BOARD pico_w)
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk")
include(${PICO_SDK_PATH}/pico_sdk_import.cmake)

project(Kolokvium C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Combined executable
add_executable(fleet_management_main
    picow_access_point.c
    mcp2515.c
    j1939.c
    dhcpserver/dhcpserver.c
    dnsserver/dnsserver.c
)
set_target_properties(fleet_management_main PROPERTIES SUFFIX ".elf")


target_include_directories(fleet_management_main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
    ${CMAKE_CURRENT_LIST_DIR}/dnsserver
)

target_link_libraries(fleet_management_main
    pico_stdlib 
    hardware_adc 
    hardware_uart 
    pico_btstack_ble 
    pico_btstack_cyw43
    hardware_spi
    hardware_rtc
    hardware_structs 
    pico_cyw43_arch_lwip_threadsafe_background 
)

# If using .gatt file and btstack config
target_include_directories(fleet_management_main PRIVATE 
${CMAKE_CURRENT_LIST_DIR}
${PICO_SDK_PATH}/lib/lwip/include
)

pico_btstack_make_gatt_header(fleet_management_main PRIVATE "${CMAKE_CURRENT_LIST_DIR}/uart.gatt")

target_compile_definitions(fleet_management_main PRIVATE IS_SERVER)

pico_add_extra_outputs(fleet_management_main)
pico_enable_stdio_usb(fleet_management_main 1)
pico_enable_stdio_uart(fleet_management_main 0)
