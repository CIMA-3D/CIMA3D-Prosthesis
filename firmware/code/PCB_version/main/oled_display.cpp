#include "oled_display.h"
#include <stdio.h>
#include <string.h>

// Envolvemos el include de la librería en C puro para que no dé errores con el C++ de tu proyecto
extern "C" {
    #include "ssd1306.h"
}

// Pines I2C de tu nueva PCB
#define SDA_PIN 8
#define SCL_PIN 9

static SSD1306_t dev;

void initOLED() {
    printf("Inicializando OLED I2C...\n");
    
    // Inicializamos el bus I2C nativo de ESP-IDF
    i2c_master_init(&dev, SDA_PIN, SCL_PIN, -1);
    
    // Inicializar pantalla (128x64). Si se viera aplastada, cambia el 64 por 32.
    ssd1306_init(&dev, 128, 64);
    
    // Limpiamos pantalla e imprimimos el mensaje de inicio
    ssd1306_clear_screen(&dev, false);
    
    // Formato: dispositivo, línea (0-7), texto, longitud del texto, color invertido (false/true)
    ssd1306_display_text(&dev, 2, (char*)" Protesis V2 ", 13, false);
    ssd1306_display_text(&dev, 4, (char*)" Iniciando... ", 14, false);
}

// Asegúrate de que los "estados" coinciden con tu enum o variables (ej. 2 = Alta, 1 = Media, 0 = Baja)
void updateOLED(const char* statusStr, int batteryState) {
    ssd1306_clear_screen(&dev, false); // Limpia la pantalla
    
    // 1. Mostrar estado de la prótesis (ej: "ACTIVA", "PINZA", etc.) en la primera línea (página 0)
    ssd1306_display_text(&dev, 0, (char*)statusStr, strlen(statusStr), false);
    
    // 2. Traducir el estado de la batería a texto
    char batStr[16];
    if (batteryState == 2) { 
        sprintf(batStr, "BAT: HIGH");
    } else if (batteryState == 1) { 
        sprintf(batStr, "BAT: MID ");
    } else { 
        sprintf(batStr, "BAT: LOW!");
    }
    
    // 3. Mostrar la batería un par de líneas más abajo (página 2 o 3)
    ssd1306_display_text(&dev, 3, batStr, strlen(batStr), false);
}

void showMessageOLED(const char* message) {
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 3, (char*)message, strlen(message), false);
}