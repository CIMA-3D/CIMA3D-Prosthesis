# CIMA3D – Prosthesis

Este repositorio contiene el codgio central del proyecto CIMA3D para el desarrollo de una prótesis mioeléctrica modular.
Es el codigo creado para la prótesis que fue a la competencia de valencia en el año 2026

## Estructura general
Este repositorio integra:
- Firmware/ → Código embebido(ESP32)
    - creado para una esp32 s3 con espidf
- Machine Learning / → Modelos y datasets
    - archivos .tflite

## Documentacion 
La mayor parte de documentacion del este proyecto esta en otro repositorio [Documentacion general](https://github.com/CIMA-3D/Documentacion_general.git)

Se realizo de este modo para poder mantener un buen flujo de verciones del codigo

## Flujo de trabajo
- codigo listo para compilar *-main
- Desarrollo, Integración y test en *-develop
- Versionado explícito *-feature
