# CIMA3D Prosthesis Valencia 2025

Este repositorio contiene el codgio central del proyecto CIMA3D para el desarrollo de una prótesis mioeléctrica modular.
Es el codigo creado para la prótesis que fue a la competencia de valencia en el año 2026

## Separacion de codigo
El otro proyecto que tenemos es la misma protesis pero no utiliza una placa de topos sino que usa PCB de 8 capas creada por el equipo de cima, esta tiene una arquitectura diferente y por eso decidimos separ el proyecto en 2 repositorios

**Codigo para segunda protesis en PCB**
**[CIMA3D-Prosthesis-PCB](https://github.com/CIMA-3D/CIMA3D-Prosthesis-PCB.git)**

## Estructura general
Este repositorio integra:
- Firmware/ → Código embebido(ESP32)
    - creado para una esp32 s3 con espidf
- Machine Learning / → Modelos y datasets
    - archivos .tflite

# Firmware

Código embebido de la prótesis.

Plataforma principal:
- ESP32

Incluye control de motores, adquisición EMG y lógica de control.

## Documentacion 
La mayor parte de documentacion del este proyecto esta en otro repositorio **[Documentacion general](https://github.com/CIMA-3D/Documentacion_general.git)**

Se realizo de este modo para poder mantener un buen flujo de verciones del codigo

## Flujo de trabajo
- codigo listo para compilar *-main
- Desarrollo, Integración y test en *-develop
- Versionado explícito *-feature
