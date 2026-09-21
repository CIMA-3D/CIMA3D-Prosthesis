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
        "oled_display.cpp"


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
        ssd1306
        SparkFun_SX1509
        Adafruit_ADS1X15
        
)
