SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ../components/MAX30102/max30102_luo.c
    ../components/MAX30102/MLX90614_API.c
    ../components/MAX30102/MLX90614_SMBus_Driver.c
    ../components/DHT11/DHT11_luo.c)
# 添加目录到 include 路径
target_include_directories(ui PUBLIC ../components/MAX30102/)
add_library(ui ${SOURCES})