#pragma once

inline constexpr float nn_scaler_mean[5] = {
    4.08170260216847414e+00f,  // f1: envelope instantaneo ch0
    3.40868627189324469e+00f,  // f2: envelope instantaneo ch1
    4.08106705865999508e+00f,  // f3: media 128 muestras ch0
    3.40777819836636375e+00f,  // f4: media 128 muestras ch1
    1.15050854336285524e+00f   // f5: |media_ch0 - media_ch1|
};

inline constexpr float nn_scaler_std[5] = {
    2.99821225663334623e+00f,
    1.71049809953527587e+00f,
    1.96361540338433804e+00f,
    1.28817599625346868e+00f,
    1.08914417010890685e+00f
};

extern const unsigned char nn_model[];
inline constexpr unsigned long nn_model_size = 3288;

inline constexpr int nn_input_size[2] = {1,	5};
inline constexpr int nn_output_size[1] = {6};
inline constexpr int nn_num_features = 5;
inline constexpr int nn_one_hot_size = 0;
inline constexpr int nn_history_length = 1;
inline constexpr int nn_window_size = 128;
inline constexpr int nn_step_size = 64;
