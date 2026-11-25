# Control de acceso a planta (humanos y robots) con ESP32

El código está en `v1_con_oled/`: pantalla OLED SSD1306 128x64 en la maqueta y dashboard web en el celular o el PC (el ESP32 crea su propia red WiFi).

Compila sin errores con **esp32 by Espressif 3.3.8**, placa **ESP32 Dev Module**. Librerías externas: **Adafruit SSD1306** y **Adafruit GFX Library**.

---

## 1. Materiales

| Componente | Cantidad | Nota |
|---|---|---|
| ESP32 DevKit V1 (WROOM-32) | 1 | No sirve un WROVER (usa los GPIO16/17) |
| Módulo cámara OV7670 **sin FIFO** | 1 | El de 18 pines y sin chip AL422B |
| Sensor ultrasónico HC-SR04 | **2** | Uno por puerta. **Hay que conseguir otro** |
| Servomotor SG90 / MG90S | **2** | Uno por puerta. **Hay que conseguir otro** |
| Pulsador | 1 | Emergencia. El reset es el botón **BOOT** de la placa |
| Buzzer **activo** 5V/3.3V | 1 | Para la señal sonora de los robots (func. adicional 3) |
| OLED SSD1306 0.96" I2C | 1 | Panel local |
| Resistencias 1 kΩ y 2 kΩ | 2 + 2 | Divisores de voltaje para los ECHO (5V → 3.3V) |
| Resistencias 4.7 kΩ | 2 | Pull-ups de SIOD/SIOC de la cámara |
| Fuente de 5V y 2A | 1 | Para los servos; GND común con el ESP32 |

> El enunciado pide una puerta de entrada y otra de salida. Con un solo ultrasonido y un solo servo no se puede saber si alguien entra o sale. Por eso hacen falta 2 de cada uno.

---

## 2. Conexiones

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
| **OLED SSD1306** | VCC / GND | 3V3 / GND |
| | SDA / SCL | 21 / 22 |
| **Ultrasonido ENTRADA** | VCC / GND | 5V (VIN) / GND |
| | TRIG | 23 |
| | ECHO → divisor 1k/2k | 36 (VP) |
| **Ultrasonido SALIDA** | VCC / GND | 5V / GND |
| | TRIG | 23 (compartido con la entrada) |
| | ECHO → divisor 1k/2k | 39 (VN) |
| **Servo ENTRADA** | señal (naranja) | 25 |
| **Servo SALIDA** | señal (naranja) | **15** |
| Servos | rojo / marrón | 5V de la fuente externa / GND común |
| **Pulsador de emergencia** | un lado / otro lado | **12** / GND (usa el pull-up interno) |
| **Pulsador de reset** | — | Botón **BOOT** de la placa (GPIO 0). No se cablea nada |
| **Buzzer activo** | + / − | 2 / GND (también parpadea el LED azul de la placa) |

**Divisor de voltaje para el ECHO** (el HC-SR04 saca 5V):
`ECHO ── 1kΩ ──┬── GPIO 36 (o 39)`
`              └── 2kΩ ── GND`

**Notas de cableado**
- Se usan todos los GPIO libres de la placa de 30 pines. Por eso los dos ultrasonidos comparten el TRIG. El código lee las puertas por turnos (una cada 40 ms), así que solo escucha el ECHO de una a la vez.
- **El servo de salida va en el GPIO 15, no en el 12.** El GPIO 12 es un pin de arranque: si está en HIGH al encender, el ESP32 no arranca o no deja cargar el programa. En el 12 va el pulsador, que solo lo lleva a GND y no molesta.
- Al encender, el servo del GPIO 15 puede dar un pequeño salto. Es normal: la placa saca unos pulsos por ese pin durante el arranque.
- No mantengas pulsado BOOT al encender o al pulsar EN: así la placa entra en modo de carga. Con el programa ya corriendo, BOOT funciona como botón de reset.
- Mantén cortos los cables de la cámara (menos de 15 cm), sobre todo PCLK y XCLK.

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

- Al arrancar, el sistema **mide la distancia al piso**. La zona debe estar vacía en ese momento. Si no lo estaba, recalibra con el botón **Calibrar sensores** del dashboard o el comando `cal`, sin reiniciar.
- `altura del objeto = distancia al piso − distancia medida`
- Si la altura es ≥ `UMBRAL_HUMANO_CM` (8 cm por defecto), es **HUMANO**. Si es menor, es **ROBOT**.
- En la maqueta, los humanos deben ser más altos (unos 10–12 cm) y los robots más bajos (unos 4–6 cm).
- Ajusta los valores al inicio del código (sección **CONFIGURACIÓN**):
  `ALTURA_MIN_PRESENCIA_CM`, `UMBRAL_HUMANO_CM`, `SERVO_ABIERTO_GRADOS`, `SERVO_CERRADO_GRADOS`.
- Con el comando `d` en el Monitor Serie ves la altura que mide cada sensor. Úsalo para calibrar.

---

## 4. Cargar el programa

1. Arduino IDE → Gestor de placas → **esp32 by Espressif Systems 3.x**.
2. En el Gestor de librerías: **Adafruit SSD1306** y **Adafruit GFX Library**. Ya quedaron instaladas en este PC.
3. Placa: **ESP32 Dev Module**. Partition scheme: *Default 4MB with spiffs*. Las fotos se guardan en LittleFS.
4. Abre `v1_con_oled/v1_con_oled.ino` y súbelo.
5. Abre el Monitor Serie a **115200 baudios**.

---

## 5. Uso

### Dashboard
1. Conéctate a la red WiFi **`Fabrica-ESP32`** (clave `fabrica123`).
2. Abre **http://192.168.4.1** (o http://fabrica.local).
3. Si quieres que el ESP32 también entre a tu WiFi, llena `STA_SSID` y `STA_PASS` en el código. La IP aparece en el Monitor Serie.

El dashboard muestra:
- Humanos y robots dentro, total sobre 12 y estado del sistema.
- Las puertas (abierta/cerrada), con el botón **Abrir** para la apertura remota (func. adicional 2).
- Una tarjeta por cada trabajador dentro, con **foto, ID, tipo y tiempo dentro**. Se pone en rojo si pasa de 90 s. Cada tarjeta tiene el botón **Marcar que sale** (ver sección 6).
- El historial de accesos (foto, ID, tipo, hora de entrada y salida, permanencia) y un registro de eventos.
- Los botones **Emergencia**, **Reset**, **Probar cámara** y **Calibrar sensores**.

### Funcionamiento automático
| Situación | Qué hace el sistema |
|---|---|
| Alguien llega a la **entrada** | Mide la altura (humano o robot), toma la foto, le asigna un ID `T01`–`T12`, inicia el contador y abre la puerta |
| La persona pasa | La puerta cierra 1.5 s después de que la zona queda libre (mínimo 3 s abierta) |
| Alguien llega a la **salida** | Clasifica humano o robot, identifica quién sale (marcado o el más antiguo de ese tipo), detiene su contador y abre la puerta |
| Entra o sale un **robot** | 3 pitidos cortos (func. adicional 3) |
| Ya hay 12 dentro | No abre la entrada, da un pitido largo y avisa "Planta llena" |
| Alguien pasa de 90 s | Alerta en la OLED y el dashboard, más 2 pitidos |
| Pulsador de emergencia (pulsación **corta**) | **EMERGENCIA**: abre ambas puertas y suena la alarma. Las puertas quedan abiertas, pero **se sigue contando** quién entra y sale |
| Botón **BOOT**, o pulsador de emergencia **mantenido 3 s** | **RESET**: cierra las puertas y pone en cero los trabajadores y los contadores de tiempo |

### Comandos del Monitor Serie (para probar sin sensores)
`eh` / `er` = entra un humano / robot · `sh` / `sr` = sale un humano / robot · `em` = emergencia · `rs` = reset · `st` = estado · `d` = distancias medidas · `cal` = recalibrar sensores · `p3` = marcar a T03 como próxima salida

---

## 6. Decisiones de diseño (para la sustentación)

- **Identidad:** el ID (`T01`–`T12`) se asigna al entrar y queda asociado a la foto y al contador. El enunciado lo permite ("identificador definido por Ud.").
- **¿Quién sale?** El ultrasonido no sabe quién es la persona que sale. Hay dos formas de resolverlo:
  1. **Marcarlo:** en el dashboard se pulsa **Marcar que sale** en la tarjeta del trabajador (o `p3` en el Monitor Serie). La próxima salida se le asigna a él. La marca vale 60 s (`VIGENCIA_MARCA_SALIDA_MS`).
  2. **Sin marca:** se asume que sale el trabajador **del mismo tipo que lleva más tiempo dentro** (FIFO por tipo).
  Mejora posible: un lector RFID RC522 con una tarjeta por trabajador.
- **Seguridad:** la puerta de salida **siempre** se abre, aunque no haya registro de esa persona. Nunca deja a nadie encerrado.
- **Emergencia:** las puertas quedan abiertas, pero los sensores siguen contando. Así el dashboard muestra quién sigue dentro durante la evacuación.
- **Dos pulsadores:** el enunciado pide un botón de emergencia y uno de reset. El de reset es el botón BOOT que ya trae la placa, porque no quedan GPIO libres para otro.
- **Cámara:** la OV7670 no genera JPEG. El ESP32 captura en 160×120 RGB565, lo convierte a JPEG por software y lo guarda en la memoria flash (LittleFS). Guarda las últimas 40 fotos. Si la cámara falla, el resto del sistema sigue funcionando y el dashboard muestra "sin cámara". Si los colores salen raros, pon `CAM_INVERTIR_BYTES = true`.
- **Sin bloqueos:** la lógica de puertas, botones, buzzer, pantalla y servidor web corre en un `loop()` con máquinas de estado. Estados de cada puerta: `CERRADA → ABIERTA → REARME → CERRADA`. Lo único que tarda es la llegada de alguien (unos 0.2 s de medición más la foto); durante ese tiempo el buzzer sigue sonando.
- **Servos y cámara:** los servos usan PWM LEDC en los canales 4 y 5. La cámara usa el timer 0 y el canal 0 para su reloj XCLK. Por eso no chocan entre sí.
- **Acceso al dashboard:** los controles no piden usuario. La protección es la clave de la red WiFi del ESP32, suficiente para una maqueta.
