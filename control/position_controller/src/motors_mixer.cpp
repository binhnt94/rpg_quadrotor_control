#include "position_controller/motors_mixer.hpp"
#include <algorithm>

MotorsAllocator::MotorsAllocator() : _roll_factor{}, _pitch_factor{}, _yaw_factor{} {
    if (FRAME == Frame::MOTOR_FRAME_TYPE_X) {
        add_motor(Motors::AP_MOTORS_MOT_1, 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
        add_motor(Motors::AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
        add_motor(Motors::AP_MOTORS_MOT_4, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
        add_motor(Motors::AP_MOTORS_MOT_2, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
    }
}

void MotorsAllocator::add_motor_raw(int motor_num, double roll_fac, double pitch_fac, double yaw_fac) {
    _roll_factor[motor_num] = roll_fac;
    _pitch_factor[motor_num] = pitch_fac;
    _yaw_factor[motor_num] = yaw_fac;
}

void MotorsAllocator::add_motor(Motors motor_num, double roll_pitch_factor_in_degrees, int yaw_factor) {
    add_motor_raw(
        static_cast<int>(motor_num),
        std::cos(roll_pitch_factor_in_degrees * M_PI / 180.0),
        std::sin(roll_pitch_factor_in_degrees * M_PI / 180.0),
        yaw_factor
    );
}

std::array<double, MotorsAllocator::AP_MOTORS_MAX_NUM_MOTORS> MotorsAllocator::update_motors_allocation(const std::array<double, 4>& des_CTBT) {
    return output_armed_stabilizing(des_CTBT);
}

std::array<double, MotorsAllocator::AP_MOTORS_MAX_NUM_MOTORS> MotorsAllocator::output_armed_stabilizing(const std::array<double, 4>& des_CTBT) {
    std::array<double, AP_MOTORS_MAX_NUM_MOTORS> _thrust_rpyt_out{};
    double throttle_thrust = des_CTBT[0];
    double roll_thrust = des_CTBT[1];
    double pitch_thrust = des_CTBT[2];
    double yaw_thrust = des_CTBT[3];

    double yaw_allowed = 1.0;
    double rpy_scale = 1.0;

    throttle_thrust = std::clamp(throttle_thrust, 0.0, 1.0);

    double throttle_thrust_best_rpy = 0.5;

    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
        if (yaw_thrust * _yaw_factor[i] > 0.0) {
            double unused_range = std::abs(std::max(1.0 - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]), 0.0) / _yaw_factor[i]);
            yaw_allowed = std::min(yaw_allowed, unused_range);
        } else {
            double unused_range = std::abs(std::max(throttle_thrust_best_rpy + _thrust_rpyt_out[i], 0.0) / _yaw_factor[i]);
            yaw_allowed = std::min(yaw_allowed, unused_range);
        }
    }

    yaw_thrust = std::clamp(yaw_thrust, -yaw_allowed, yaw_allowed);

    double rpy_low = 1.0;
    double rpy_high = -1.0;
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _thrust_rpyt_out[i] += yaw_thrust * _yaw_factor[i];
        rpy_low = std::min(rpy_low, _thrust_rpyt_out[i]);
        rpy_high = std::max(rpy_high, _thrust_rpyt_out[i]);
    }

    if (rpy_high - rpy_low > 1.0) {
        rpy_scale = 1.0 / (rpy_high - rpy_low);
    }
    if (1 + rpy_low < 0) {
        rpy_scale = std::min(rpy_scale, -1.0 / rpy_low);
    }

    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    double thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0) {
        thr_adj = 0.0;
    } else {
        thr_adj = std::clamp(thr_adj, 0.0, 1.0 - (throttle_thrust_best_rpy + rpy_high));
    }

    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + (rpy_scale * _thrust_rpyt_out[i]);
    }

    for (auto& thrust : _thrust_rpyt_out) {
        thrust *= 300;
    }

    return _thrust_rpyt_out;
}