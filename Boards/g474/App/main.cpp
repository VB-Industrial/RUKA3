#include "app.h"

#include "tim.h"
#include "cordic.h"

void setup_timers() {
    HAL_TIM_Base_Start_IT(&htim3);
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

static uint32_t millis_k = 0;

#ifdef DEBUG
static volatile encoder_data _raw_angle;
static volatile float _angle;
static volatile float _velocity;
static volatile float _torque;
#endif
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
#ifndef ENABLE_CALIBRATION
        heartbeat();
#endif
    }
    else if (htim->Instance == TIM7) {
        millis_k += 1;
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
    motor.init();
    HAL_Delay(200);  //sanity delay
    motor.start();

    setup_timers();

#ifdef ENABLE_CALIBRATION
    static CalibrationData calibration_data;
    static std::array<std::byte, CALIBRATION_BUFF_SIZE> calibration_buffer{};
    motor.calibrate(calibration_data, calibration_buffer.data(), calibration_buffer.size());
    motor.apply_calibration(calibration_data);
    
#endif

     while(true) {
#ifndef ENABLE_CALIBRATION
        comms_loop(millis_k);
#endif
    }
}
