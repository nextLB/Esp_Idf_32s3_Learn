# ============================================================
# 程序选择开关（每个源文件都独立自带 app_main）
#   1 = pca9685_servo_control.c          （多路舵机单测）
#   2 = pca9685_sg90_speed_calibration.c （SG90 速度标定）
#   3 = simple_walking.c                 （桌面机器狗走/退 · 旧版）
#   4 = four_legged_walking.c            （四足狗 前进BOUND+后退DIAGONAL · 封装）
# ============================================================
set(SERVO_APP 4)

if(SERVO_APP EQUAL 1)
    set(SERVO_SRCS "pca9685_servo_control.c")
elseif(SERVO_APP EQUAL 2)
    set(SERVO_SRCS "pca9685_sg90_speed_calibration.c")
elseif(SERVO_APP EQUAL 3)
    set(SERVO_SRCS "simple_walking.c")
elseif(SERVO_APP EQUAL 4)
    set(SERVO_SRCS "four_legged_walking.c")
else()
    message(FATAL_ERROR "SERVO_APP 只能设为 1/2/3/4，当前值: ${SERVO_APP}")
endif()

message(STATUS "SERVO_APP = ${SERVO_APP}, 编译源文件: ${SERVO_SRCS}")

idf_component_register(
    SRCS
        ${SERVO_SRCS}
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        driver
        esp_timer
        esp_common
        log
        freertos
)