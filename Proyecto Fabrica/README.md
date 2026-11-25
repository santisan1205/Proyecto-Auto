# Control de acceso a planta (humanos y robots) con ESP32

Hay dos versiones. Las dos tienen la misma lógica y el mismo dashboard web:

| Carpeta | Pantalla | Panel local | Librerías externas |
|---|---|---|---|
| `v1_con_oled/` | OLED SSD1306 128x64 (I2C) | OLED + dashboard | Adafruit SSD1306, Adafruit GFX |
| `v2_sin_oled/` | No tiene | Dashboard web (el ESP32 crea su propia red WiFi) | Ninguna |

Las dos versiones compilan sin errores con **esp32 by Espressif 3.3.8**, placa **ESP32 Dev Module**.

---

## 1. Materiales

| Componente | Cantidad | Nota |
|---|---|---|
| ESP32 DevKit V1 (WROOM-32) | 1 | No sirve un WROVER (usa los GPIO16/17) |
| Módulo cámara OV7670 **sin FIFO** | 1 | El de 18 pines y sin chip AL422B |
| Sensor ultrasónico HC-SR04 | **2** | Uno por puerta. **Hay que conseguir otro** |
| Servomotor SG90 / MG90S | **2** | Uno por puerta. **Hay que conseguir otro** |
| Pulsador | 1 (+1 opcional en V2) | Corto = emergencia, 3 s = reset |
| Buzzer **activo** 5V/3.3V | 1 | Para la señal sonora de los robots (func. adicional 3) |
| OLED SSD1306 0.96" I2C | 1 | Solo para la V1 |
| Resistencias 1 kΩ y 2 kΩ | 2 + 2 | Divisores de voltaje para los ECHO (5V → 3.3V) |
| Resistencias 4.7 kΩ | 2 | Pull-ups de SIOD/SIOC de la cámara |
| Fuente de 5V y 2A | 1 | Para los servos; GND común con el ESP32 |

> El enunciado pide una puerta de entrada y otra de salida. Con un solo ultrasonido y un solo servo no se puede saber si alguien entra o sale. Por eso hacen falta 2 de cada uno.

---

## 2. Conexiones

### Comunes a V1 y V2

| Elemento | Pin del módulo | GPIO del ESP32 |
|---|---|---|
| **OV7670** | 3V3 / GND | 3V3 / GND (**nunca a 5V**) |
| | SIOC (SCL) | 27 (+4.7 kΩ a 3V3) |
| | SIOD (SDA) | 26 (+4.7 kΩ a 3V3) |
| | VSYNC | 34 |
| | HREF | 35 |
| | PCLK | 33 |
| | XCLK | 32 |
| | D0 / D1 / D2 / D3 | 4 / 5 / 18 / 19 |
| | D4 / D5 / D6 / D7 | 13 / 14 / 16 / 17 |
| | RESET | 3V3 |
| | PWDN | GND |
| **Ultrasonido ENTRADA** | VCC / GND | 5V (VIN) / GND |
| | TRIG | 23 |
| | ECHO → divisor 1k/2k | 36 (VP) |
| **Ultrasonido SALIDA** | VCC / GND | 5V / GND |
| | TRIG | **V1: 23 (compartido)** · **V2: 22** |
| | ECHO → divisor 1k/2k | 39 (VN) |
| **Servo ENTRADA** | señal (naranja) | 25 |
| **Servo SALIDA** | señal (naranja) | 12 |
| Servos | rojo / marrón | 5V de la fuente externa / GND común |
| **Pulsador** | un lado / otro lado | 15 / GND (usa el pull-up interno) |
| **Buzzer activo** | + / − | 2 / GND (también parpadea el LED azul de la placa) |

### Solo V1 (OLED)

| OLED | ESP32 |
|---|---|
| VCC / GND | 3V3 / GND |
| SDA | 21 |
| SCL | 22 |

### Solo V2 (opcional)

| Elemento | ESP32 |
|---|---|
| Segundo pulsador de RESET | 21 / GND |

**Divisor de voltaje para el ECHO** (el HC-SR04 saca 5V):
`ECHO ── 1kΩ ──┬── GPIO 36 (o 39)`
`              └── 2kΩ ── GND`

**Notas de cableado**
- En la V1 se usan los 23 GPIO disponibles. Por eso los dos ultrasonidos comparten el TRIG. El código mide las puertas de forma alternada, así que funciona bien.
- Mantén cortos los cables de la cámara (menos de 15 cm), sobre todo PCLK y XCLK.
- Si la placa no arranca al cargar el programa, desconecta el servo del GPIO12 mientras subes el código.

---

## 3. Montaje de los sensores (cómo distingue humano de robot)

Cada ultrasonido va **arriba, en la zona de acceso antes de la puerta, mirando hacia el piso**:

```
        [HC-SR04]  ← a ~20 cm del piso
            |
            |  distancia medida
            v
         ┌──┐
         │  │  ← trabajador (altura = piso − distancia)
   ──────┴──┴────────  piso
```

- Al arrancar, el sistema **mide la distancia al piso**. La zona debe estar vacía en ese momento.
- `altura del objeto = distancia al piso − distancia medida`
- Si la altura es ≥ `UMBRAL_HUMANO_CM` (8 cm por defecto), es **HUMANO**. Si es menor, es **ROBOT**.
- En la maqueta, los humanos deben ser más altos (unos 10–12 cm) y los robots más bajos (unos 4–6 cm).
- Ajusta los valores al inicio del código (sección **CONFIGURACIÓN**):
  `ALTURA_MIN_PRESENCIA_CM`, `UMBRAL_HUMANO_CM`, `SERVO_ABIERTO_GRADOS`, `SERVO_CERRADO_GRADOS`.
- Con el comando `d` en el Monitor Serie ves la altura que mide cada sensor. Úsalo para calibrar.

---

## 4. Cargar el programa

1. Arduino IDE → Gestor de placas → **esp32 by Espressif Systems 3.x**.
2. Solo para la V1, en el Gestor de librerías: **Adafruit SSD1306** y **Adafruit GFX Library**. Ya quedaron instaladas en este PC.
3. Placa: **ESP32 Dev Module**. Partition scheme: *Default 4MB with spiffs*. Las fotos se guardan en LittleFS.
4. Abre `v1_con_oled/v1_con_oled.ino` o `v2_sin_oled/v2_sin_oled.ino` y súbelo.
5. Abre el Monitor Serie a **115200 baudios**.

---

## 5. Uso

### Dashboard (las dos versiones)
1. Conéctate a la red WiFi **`Fabrica-ESP32`** (clave `fabrica123`).
2. Abre **http://192.168.4.1** (o http://fabrica.local).
3. Si quieres que el ESP32 también entre a tu WiFi, llena `STA_SSID` y `STA_PASS` en el código. La IP aparece en el Monitor Serie.

El dashboard muestra:
- Humanos y robots dentro, total sobre 12 y estado del sistema.
- Las puertas (abierta/cerrada), con el botón **Abrir** para la apertura remota (func. adicional 2).
- Una tarjeta por cada trabajador dentro, con **foto, ID, tipo y tiempo dentro**. Se pone en rojo si pasa de 90 s.
- El historial de accesos (foto, ID, tipo, hora de entrada y salida, permanencia) y un registro de eventos.
- Los botones **Emergencia**, **Reset** y **Probar cámara**.

### Funcionamiento automático
| Situación | Qué hace el sistema |
|---|---|
| Alguien llega a la **entrada** | Mide la altura (humano o robot), toma la foto, le asigna un ID `T01`–`T12`, inicia el contador y abre la puerta |
| La persona pasa | La puerta cierra 1.5 s después de que la zona queda libre (mínimo 3 s abierta) |
| Alguien llega a la **salida** | Clasifica humano o robot, detiene su contador, lo saca del conteo y abre la puerta |
| Entra o sale un **robot** | 3 pitidos cortos (func. adicional 3) |
| Ya hay 12 dentro | No abre la entrada, da un pitido largo y avisa "Planta llena" |
| Alguien pasa de 90 s | Alerta en la OLED y el dashboard, más 2 pitidos |
| Pulsación **corta** del botón | **EMERGENCIA**: abre ambas puertas, suena la alarma y las puertas quedan abiertas |
| Botón **mantenido 3 s** | **RESET**: cierra las puertas y pone en cero los trabajadores y los contadores de tiempo |

### Comandos del Monitor Serie (para probar sin sensores)
`eh` / `er` = entra un humano / robot · `sh` / `sr` = sale un humano / robot · `em` = emergencia · `rs` = reset · `st` = estado · `d` = distancias medidas

---

## 6. Decisiones de diseño (para la sustentación)

- **Identidad:** el ID (`T01`–`T12`) se asigna al entrar y queda asociado a la foto y al contador. El enunciado lo permite ("identificador definido por Ud.").
- **¿Quién sale?** Sin un lector de tarjetas no se puede saber quién es la persona que sale. El sistema asume que sale el trabajador **del mismo tipo que lleva más tiempo dentro** (FIFO por tipo). Mejora posible: un lector RFID RC522 con una tarjeta por trabajador.
- **Seguridad:** la puerta de salida **siempre** se abre, aunque no haya registro de esa persona. Nunca deja a nadie encerrado.
- **Cámara:** la OV7670 no genera JPEG. El ESP32 captura en 160×120 RGB565, lo convierte a JPEG por software y lo guarda en la memoria flash (LittleFS). Guarda las últimas 40 fotos. Si la cámara falla, el resto del sistema sigue funcionando y el dashboard muestra "sin cámara".
- **Sin bloqueos:** la lógica de puertas, botones, buzzer, pantalla y servidor web corre en un `loop()` no bloqueante, con máquinas de estado. Estados de cada puerta: `CERRADA → ABIERTA → REARME → CERRADA`.
- **Servos y cámara:** los servos usan PWM LEDC en los canales 4 y 5. La cámara usa el timer 0 y el canal 0 para su reloj XCLK. Por eso no chocan entre sí.
