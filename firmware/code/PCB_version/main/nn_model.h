#pragma once

// --- SCALER ---
extern const float nn_scaler_mean[5];
extern const float nn_scaler_std[5];

// --- MODELO ---
extern const unsigned char nn_model[];
extern const unsigned int nn_model_len;

// --- CONFIGURACION DE LA RED ---
inline constexpr int nn_input_size[2] = {1, 9};
inline constexpr int nn_output_size[1] = {4};
inline constexpr int nn_num_features = 5;
inline constexpr int nn_one_hot_size = 4;
inline constexpr int nn_history_length = 1;
inline constexpr int nn_window_size = 128;
inline constexpr int nn_step_size = 64;