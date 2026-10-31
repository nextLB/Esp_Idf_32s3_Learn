# ============================================================
# 程序选择开关
#   1 = pca9685_servo_control.c          （多路舵机测试）
#   2 = pca9685_sg90_speed_calibration.c （SG90 速度标定）
# ============================================================
set(SERVO_APP 1)

if(SERVO_APP EQUAL 1)
    set(SERVO_SRCS "pca9685_servo_control.c")
elseif(SERVO_APP EQUAL 2)
    set(SERVO_SRCS "pca9685_sg90_speed_calibration.c")
else()
    message(FATAL_ERROR "SERVO_APP 只能设为 1 或 2，当前值: ${SERVO_APP}")
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