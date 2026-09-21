idf_component_register(
    SRCS
        "main.cpp"
        "AdcSingleRead.cpp"
        "emg_continuous_sampling.cpp"
        "globals.cpp"
        "movements.cpp"
        "NeuralNetwork.cpp"
        "secondary_sensors.cpp"
        "uart_command_handler.cpp"
        "nn_model.cc"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        esp_timer
        esp_adc
        freertos
        Servo
        EMGController
        NonBlockingTimer
        PwmController
        SmartLeds
        esp-tflite-micro
)
