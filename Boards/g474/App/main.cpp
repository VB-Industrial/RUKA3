#include "app.h"

#include "tim.h"
#include "cordic.h"

#ifdef ENABLE_CALIBRATION
#error "Legacy open-loop calibration is not supported by the G474 current commissioning image"
#endif

void setup_timers() {
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim1);
}

void setup_cordic() {
    CORDIC_ConfigTypeDef cordic_config {
        .Function = CORDIC_FUNCTION_COSINE,
        .Scale = CORDIC_SCALE_0,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
        .NbWrite = CORDIC_NBWRITE_1,
        .NbRead = CORDIC_NBREAD_2,
        .Precision = CORDIC_PRECISION_6CYCLES
    };
    HAL_CORDIC_Configure(&hcordic, &cordic_config);
}

static volatile uint32_t millis_k = 0;

#ifdef DEBUG
static volatile encoder_data _raw_angle;
static volatile float _angle;
static volatile float _velocity;
static volatile float _torque;
#endif
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // CAN allocation/publication belongs to the main loop, not an ISR.
    }
    else if (htim->Instance == TIM7) {
        millis_k = millis_k + 1;
    }
    else if (htim->Instance == TIM1) {
        FOC& motor = get_motor();
#ifdef DEBUG
        _angle = motor.get_angle();
        _velocity = motor.get_velocity();
        _torque = motor.get_torque();
        _raw_angle = motor.get_encoder().get_value();
#endif
#ifndef ENABLE_CALIBRATION
        motor.update();
#endif
    }
}

micros __attribute__((optimize("O0"))) micros_64() {
    return (micros)(millis_k * 1000u + __HAL_TIM_GetCounter(&htim7));
}


[[noreturn]] void app() {
    setup_cordic();
#ifndef ENABLE_CALIBRATION
    setup_cyphal_comms();
    setup_can();
#endif
    FOC& motor = get_motor();
    HAL_IMPORTANT(motor.init())
    motor.set_voltage_point(0.0F);
    // No power-stage enable at startup. Initialize sensing/fusion first.
    for (unsigned i = 0; i < 32; ++i) motor.update();
    setup_motor_control();

    setup_timers();

#ifdef ENABLE_CALIBRATION
    static CalibrationData calibration_data;
    static std::array<std::byte, CALIBRATION_BUFF_SIZE> calibration_buffer{};
    motor.calibrate(calibration_data, calibration_buffer.data(), calibration_buffer.size());
    motor.apply_calibration(calibration_data);
    
#endif

     while(true) {
#ifndef ENABLE_CALIBRATION
        comms_loop(HAL_GetTick());
#endif
    }
}
