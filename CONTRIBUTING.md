# Contributing to CIMA3D-Prosthesis

Este documento define las reglas para contribuir al repositorio CIMA3D-Prosthesis.

---

## Versionado de piezas 3D (STL / STEP)

En impresión 3D se generan múltiples iteraciones de una misma pieza.
Para evitar confusión, se deben seguir estrictamente las siguientes normas:

### 📁 Estructura de carpetas
Cada pieza debe organizarse por versiones:

/mechanical/socket/v1/
/mechanical/socket/v2/
/mechanical/socket/prototype_YYYY_MM/

No se deben sobrescribir archivos antiguos.

---

### 📦 Contenido obligatorio por versión
Cada carpeta de versión debe contener como mínimo:

- Archivo STL
- Archivo STEP (si existe)
- Archivo CHANGELOG.md describiendo los cambios

Ejemplo de estructura:

/mechanical/socket/v2/
├── socket_v2.stl
├── socket_v2.step
└── CHANGELOG.md

---

### 📝 CHANGELOG.md
El archivo CHANGELOG.md debe describir brevemente:

- Qué se ha modificado
- Por qué se ha modificado
- Fecha
- Autor/a

---

### 🌿 Uso de ramas
Mientras el diseño esté en desarrollo:

- El trabajo mecánico se realiza en `mechanical-dev`
- Electrónica en `electronics-dev`
- Firmware en `firmware-dev`
- Machine Learning en `ml-dev`

Solo las versiones validadas se fusionan en `main`.

---

❌ No subir STL sueltos sin versión  
❌ No sobrescribir archivos existentes  
❌ No subir directamente a `main`
