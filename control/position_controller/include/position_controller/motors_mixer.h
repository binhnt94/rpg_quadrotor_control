#pragma once

#include <array>
#include <cmath>

enum class Frame {
    MOTOR_FRAME_TYPE_PLUS = 0,
    MOTOR_FRAME_TYPE_X = 1,
    MOTOR_FRAME_TYPE_V = 2,
    MOTOR_FRAME_TYPE_H = 3,
    MOTOR_FRAME_TYPE_VTAIL = 4,
    MOTOR_FRAME_TYPE_ATAIL = 5,
    MOTOR_FRAME_TYPE_PLUSREV = 6,
    MOTOR_FRAME_TYPE_Y6B = 10,
    MOTOR_FRAME_TYPE_Y6F = 11,
    MOTOR_FRAME_TYPE_BF_X = 12,
    MOTOR_FRAME_TYPE_DJI_X = 13,
    MOTOR_FRAME_TYPE_CW_X = 14,
    MOTOR_FRAME_TYPE_I = 15,
    MOTOR_FRAME_TYPE_NYT_PLUS = 16,
    MOTOR_FRAME_TYPE_NYT_X = 17,
    MOTOR_FRAME_TYPE_BF_X_REV = 18
};

enum class Motors {
    AP_MOTORS_MOT_1 = 0,
    AP_MOTORS_MOT_2 = 1,
    AP_MOTORS_MOT_3 = 2,
    AP_MOTORS_MOT_4 = 3
};

constexpr Frame FRAME = Frame::MOTOR_FRAME_TYPE_X;
constexpr int AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1;
constexpr int AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1;
constexpr int AP_MOTORS_MAX_NUM_MOTORS = 4;

class MotorsAllocator {
private:
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> _roll_factor;
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> _pitch_factor;
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> _yaw_factor;

    void add_motor_raw(int motor_num, double roll_fac, double pitch_fac, double yaw_fac);
    void add_motor(Motors motor_num, double roll_pitch_factor_in_degrees, int yaw_factor);
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> output_armed_stabilizing(const std::array<double, 4>& des_CTBT);

public:
    MotorsAllocator();
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> update(const std::array<double, 4>& des_CTBT);
};