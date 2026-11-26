/*
  =====================================================================
   PROYECTO DE AUTOMATIZACIÓN – CONTROL DE ACCESO A PLANTA
   (humanos y robots, 2 puertas, foto, contador de permanencia)

   VERSIÓN 1: CON PANTALLA OLED + DASHBOARD WEB
  =====================================================================
   Placa : ESP32 DevKit V1 (ESP32-WROOM-32, 30 o 38 pines, sin PSRAM)
   Core  : "esp32 by Espressif Systems" versión 3.x (probado con 3.3.8)
   Librerías externas (Gestor de librerías del Arduino IDE):
     - Adafruit SSD1306
     - Adafruit GFX Library
   (WiFi, WebServer, ESPmDNS, LittleFS y la cámara vienen con el core)

   Requerimientos cubiertos:
     1. Detecta presencia (ultrasonido) y abre la puerta (servo), en
        entrada y salida.
     2. Inicia un contador de permanencia por trabajador.
     3. Identifica HUMANO o ROBOT por la altura medida por el ultrasonido
        (sensor montado arriba de la zona de acceso mirando hacia abajo).
     4. Captura la foto (OV7670) y la asocia a un ID (T01..T12).
     5. Pantalla OLED local con conteo, estado de puertas, lista y avisos.
     6. Pulsador de EMERGENCIA (abre ambas puertas) + pulsador de RESET
        (botón BOOT de la placa: cierra puertas y reinicia contadores).
        Mantener el pulsador de emergencia 3 s también hace RESET.
   Funcionalidades adicionales:
     1. Dashboard web (WiFi) con ID, foto, tiempo dentro e historial.
     2. Apertura remota de las puertas desde el dashboard.
     3. Señal sonora (buzzer) cada vez que un robot entra o sale.
   Extras: alerta si alguien supera 90 s, autocalibración del
   ultrasonido (también bajo demanda), marcar desde el dashboard quién
   va a salir, conteo de entradas/salidas durante la emergencia y
   comandos por Monitor Serie para simular entradas.
  =====================================================================
*/

#include <Arduino.h>
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Este código requiere el core 'esp32 by Espressif Systems' versión 3.x"
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdarg.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "dashboard.h"

// =====================================================================
//                          CONFIGURACIÓN
// =====================================================================

// ---------------- WiFi ----------------
// El ESP32 SIEMPRE crea su propia red (Access Point). Conéctate a ella con
// el celular o PC y abre http://192.168.4.1
const char* AP_SSID   = "Fabrica-ESP32";
const char* AP_PASS   = "fabrica123";      // mínimo 8 caracteres
// Opcional: además se conecta a tu red WiFi (déjalo "" para no usarlo).
const char* STA_SSID  = "";
const char* STA_PASS  = "";
const char* MDNS_NAME = "fabrica";         // -> http://fabrica.local

// ---------------- Pines ----------------
// Cámara OV7670 (versión SIN FIFO). RESET -> 3.3V, PWDN -> GND.
#define CAM_PIN_XCLK   32
#define CAM_PIN_PCLK   33
#define CAM_PIN_VSYNC  34
#define CAM_PIN_HREF   35
#define CAM_PIN_SIOD   26   // SDA de la cámara (pull-up 4.7k a 3.3V)
#define CAM_PIN_SIOC   27   // SCL de la cámara (pull-up 4.7k a 3.3V)
#define CAM_PIN_D0     19
#define CAM_PIN_D1     18
#define CAM_PIN_D2     17
#define CAM_PIN_D3      5
#define CAM_PIN_D4      4
#define CAM_PIN_D5     16
#define CAM_PIN_D6     13
#define CAM_PIN_D7     14

// Pantalla OLED SSD1306 128x64 I2C
#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_ADDR    0x3C   // algunas pantallas usan 0x3D
#define OLED_ANCHO    128
#define OLED_ALTO      64

// Ultrasonidos HC-SR04. En esta versión los dos sensores comparten el pin
// TRIG (no quedan más pines libres). Se leen por turnos, así que solo se
// escucha el ECHO de una puerta a la vez. ECHO con divisor de voltaje 1k/2k.
#define PIN_TRIG_ENTRADA  23
#define PIN_ECHO_ENTRADA  36   // VP (solo entrada)
#define PIN_TRIG_SALIDA   23   // compartido con la entrada
#define PIN_ECHO_SALIDA   39   // VN (solo entrada)

// Servomotores (señal). Alimentarlos con 5V externos, GND común.
// El servo de salida NO va en el GPIO 12: es un pin de arranque y si está
// en HIGH al encender, el ESP32 no arranca. El GPIO 15 no tiene ese problema.
#define PIN_SERVO_ENTRADA 25
#define PIN_SERVO_SALIDA  15

// Pulsador de emergencia (a GND): corto = emergencia, mantener 3 s = reset.
// En el GPIO 12 no molesta al arranque porque el botón solo lo lleva a GND.
#define PIN_BOTON          12
// Pulsador de reset: el botón BOOT de la placa (GPIO 0). No necesita cables.
// Pon -1 para desactivarlo.
#define PIN_BOTON_RESET     0

// Buzzer ACTIVO (HIGH = suena). El LED azul de la placa también parpadea.
#define PIN_BUZZER          2

// ---------------- Parámetros del proceso ----------------
const int      MAX_TRABAJADORES          = 12;
const uint32_t TIEMPO_MAX_PERMANENCIA_MS = 90000;  // 90 s

// Clasificación por altura (en cm). El sensor está arriba mirando al piso:
//   altura del objeto = distancia al piso (calibrada) - distancia medida
// Hay dos rangos separados por una zona muerta, para que no haya dudas:
//   2.0 – 6.5 cm  -> ROBOT
//   6.5 – 9.5 cm  -> zona muerta: no se registra; se pide subir o bajar
//   9.5 cm o más  -> HUMANO
const float ALTURA_MIN_PRESENCIA_CM = 3.0;   // más alto que esto = hay alguien
const float ROBOT_MAX_CM            = 10.5;   // <= esto ROBOT
const float HUMANO_MIN_CM           = 18.5;   // >= esto HUMANO
const uint32_t REINTENTO_DUDOSA_MS  = 1000;  // vuelve a medir cada 1 s si cae en la zona muerta
// Monitor Serie: imprime todo el tiempo la distancia que mide cada ultrasonido.
// Se enciende/apaga con el comando 'dd'.
const bool     MOSTRAR_DISTANCIAS_AL_INICIO = true;
const uint32_t PERIODO_MOSTRAR_DIST_MS      = 200;   // cada cuánto imprime (5 veces por segundo)
const float ALTURA_SENSOR_DEFECTO_CM = 20.0; // si la autocalibración falla

// Puertas
const int      SERVO_CERRADO_GRADOS   = 0;
const int      SERVO_ABIERTO_GRADOS   = 90;
const uint8_t  LECTURAS_CONFIRMACION  = 3;     // lecturas seguidas para confirmar presencia
const uint32_t PERIODO_MEDICION_MS    = 40;    // se alterna entre puertas
const uint32_t PUERTA_ABIERTA_MIN_MS  = 3000;
const uint32_t PUERTA_ESPERA_LIBRE_MS = 1500;  // cierra 1.5 s después de que pasa
const uint32_t PUERTA_ABIERTA_MAX_MS  = 10000;
const uint32_t APERTURA_REMOTA_MS     = 5000;

// Salida: tiempo durante el cual vale la marca "este trabajador va a salir"
// hecha desde el dashboard o el Monitor Serie.
const uint32_t VIGENCIA_MARCA_SALIDA_MS = 60000;

// Botones
const uint32_t PULSACION_LARGA_MS = 3000;
const uint32_t ANTIRREBOTE_MS     = 30;

// Sonido
const bool     BEEP_HUMANOS            = false; // true = pitido corto también para humanos
const uint32_t DURACION_ALARMA_EMERG_MS = 10000;

// Fotos
const int     MAX_FOTOS     = 40;   // fotos guardadas en memoria (LittleFS)
const uint8_t CALIDAD_JPEG  = 80;
// Si las fotos salen con colores raros (azules/verdes cambiados o con
// "ruido" de colores), pon esto en true: invierte el orden de bytes RGB565.
const bool    CAM_INVERTIR_BYTES = false;
// Reloj que el ESP32 le da a la cámara (pin XLK). Si la foto falla con
// cables largos o en protoboard, bájalo (6000000) para que sea más tolerante.
const uint32_t CAM_XCLK_HZ = 6000000;
// Divisor del reloj interno de la cámara (registro CLKRC). El reloj de
// salida PLK vale XLK / (divisor + 1). Si la foto sale con franjas de
// colores y una línea diagonal, se están perdiendo pulsos de PLK: sube este
// valor (2, 3...) para hacer PLK más lento. Se prueba sin subir el código
// con el comando "div N" del Monitor Serie. El driver usa 1.
uint8_t camDivisorReloj = 1;

// =====================================================================
//                       TIPOS Y VARIABLES GLOBALES
// =====================================================================

enum Tipo : uint8_t { HUMANO = 0, ROBOT = 1 };

struct Trabajador {
  bool     dentro;
  char     id[4];       // "T01".."T12"
  Tipo     tipo;
  uint32_t tEntrada;    // millis() al entrar
  uint32_t visita;      // número único de visita (asocia la foto)
  bool     foto;
  bool     alerta;      // superó el tiempo máximo
};
Trabajador trabajadores[MAX_TRABAJADORES];

struct Registro {       // historial de visitas (más reciente primero)
  uint32_t visita;
  char     id[4];
  Tipo     tipo;
  uint32_t tEntrada;
  uint32_t tSalida;     // 0 si sigue dentro
  bool     dentro;
  bool     foto;
};
const int MAX_HISTORIAL = 20;
Registro historial[MAX_HISTORIAL];
int nHistorial = 0;

struct Evento { uint32_t t; char txt[48]; };
const int MAX_EVENTOS = 15;
Evento eventos[MAX_EVENTOS];
int nEventos = 0;

enum EstadoPuerta : uint8_t { P_CERRADA, P_ABIERTA, P_REARME };
struct Puerta {
  const char*  nombre;
  bool         esEntrada;
  int          pinTrig, pinEcho, pinServo, canalPwm;
  float        alturaSensor;     // distancia al piso (calibrada)
  EstadoPuerta estado;
  uint8_t      conteo;
  uint32_t     tEstado;
  uint32_t     tUltimaPresencia;
  bool         remota;
  float        ultimaAltura;
  bool         ocupada;          // solo en emergencia: hay alguien bajo el sensor
  uint32_t     tDudosa;          // última vez que la altura cayó en la zona muerta
  float        ultimaDistancia;  // última distancia medida en cm (-1 = sin eco)
};
Puerta puertas[2] = {
  { "Entrada", true,  PIN_TRIG_ENTRADA, PIN_ECHO_ENTRADA, PIN_SERVO_ENTRADA, 4, ALTURA_SENSOR_DEFECTO_CM, P_CERRADA, 0, 0, 0, false, 0, false, 0, -1 },
  { "Salida",  false, PIN_TRIG_SALIDA,  PIN_ECHO_SALIDA,  PIN_SERVO_SALIDA,  5, ALTURA_SENSOR_DEFECTO_CM, P_CERRADA, 0, 0, 0, false, 0, false, 0, -1 },
};

struct Boton {
  int pin;
  bool presionado;
  bool lecturaPrev;
  uint32_t tCambio;
  uint32_t tPresion;
  bool largoHecho;
};
Boton boton      = { PIN_BOTON,       false, false, 0, 0, false };
Boton botonReset = { PIN_BOTON_RESET, false, false, 0, 0, false };

const char* nombreTipo(Tipo t) { return t == HUMANO ? "HUMANO" : "ROBOT"; }

bool     emergencia      = false;
uint32_t tEmergencia     = 0;
bool     camaraOK        = false;
bool     mostrarDistancias = MOSTRAR_DISTANCIAS_AL_INICIO;
uint32_t tMostrarDist    = 0;
uint32_t contadorVisitas = 0;
uint32_t bootId          = 0;
uint32_t tUltimaMedicion = 0;
uint8_t  puertaTurno     = 0;
uint32_t visitaMarcadaSalida = 0;   // 0 = nadie marcado
uint32_t tMarcaSalida        = 0;

WebServer server(80);
Adafruit_SSD1306 oled(OLED_ANCHO, OLED_ALTO, &Wire, -1);
bool oledOK = false;

// =====================================================================
//                          REGISTRO DE EVENTOS
// =====================================================================
void logEvento(const char* fmt, ...) {
  for (int i = MAX_EVENTOS - 1; i > 0; i--) eventos[i] = eventos[i - 1];
  eventos[0].t = millis();
  va_list args;
  va_start(args, fmt);
  vsnprintf(eventos[0].txt, sizeof(eventos[0].txt), fmt, args);
  va_end(args);
  if (nEventos < MAX_EVENTOS) nEventos++;
  Serial.printf("[%7.1f s] %s\n", millis() / 1000.0, eventos[0].txt);
}

// =====================================================================
//                               BUZZER
// =====================================================================
// Patrones: duraciones alternando ENCENDIDO / APAGADO (ms)
const uint16_t PAT_HUMANO[]     = { 60 };
const uint16_t PAT_ROBOT[]      = { 120, 80, 120, 80, 120 };
const uint16_t PAT_ERROR[]      = { 800 };
const uint16_t PAT_ALERTA[]     = { 300, 150, 300 };
const uint16_t PAT_EMERGENCIA[] = { 400, 200 };
const uint16_t PAT_RESET[]      = { 80, 80, 300 };

const uint16_t* patron = nullptr;
uint8_t  patronLen = 0, patronIdx = 0;
uint32_t tPaso = 0;

void sonar(const uint16_t* p, uint8_t len) {
  patron = p; patronLen = len; patronIdx = 0; tPaso = millis();
  digitalWrite(PIN_BUZZER, HIGH);
}
#define SONAR(p) sonar(p, sizeof(p) / sizeof(p[0]))

void actualizarBuzzer() {
  if (!patron) {
    // La alarma de emergencia se repite durante unos segundos
    if (emergencia && millis() - tEmergencia < DURACION_ALARMA_EMERG_MS) SONAR(PAT_EMERGENCIA);
    return;
  }
  if (millis() - tPaso < patron[patronIdx]) return;
  tPaso = millis();
  patronIdx++;
  if (patronIdx >= patronLen) {
    patron = nullptr;
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }
  digitalWrite(PIN_BUZZER, (patronIdx % 2 == 0) ? HIGH : LOW);
}

// Espera sin congelar el buzzer (se usa en las mediciones que tardan)
void esperarActivo(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { actualizarBuzzer(); delay(1); }
}

// =====================================================================
//                               SERVOS
// =====================================================================
void iniciarServo(Puerta& p) {
  ledcAttachChannel(p.pinServo, 50, 14, p.canalPwm);   // 50 Hz, 14 bits
}
void moverServo(int pin, int grados) {
  grados = constrain(grados, 0, 180);
  uint32_t us   = map(grados, 0, 180, 500, 2400);       // SG90 / MG90S
  uint32_t duty = us * 16383UL / 20000UL;
  ledcWrite(pin, duty);
}

void abrirPuerta(Puerta& p, bool remota = false) {
  moverServo(p.pinServo, SERVO_ABIERTO_GRADOS);
  p.estado = P_ABIERTA;
  p.tEstado = p.tUltimaPresencia = millis();
  p.remota = remota;
  p.conteo = 0;
}

void cerrarPuerta(Puerta& p) {
  moverServo(p.pinServo, SERVO_CERRADO_GRADOS);
  p.estado = P_REARME;        // espera a que la zona quede libre
  p.tEstado = millis();
  p.conteo = 0;
  p.remota = false;
}

// =====================================================================
//                            ULTRASONIDO
// =====================================================================
float medirDistanciaCm(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 12000UL);   // máx ~2 m
  if (us == 0) return -1;
  return us * 0.0343f / 2.0f;
}

// Altura del objeto bajo el sensor (0 si no hay nada)
float medirAltura(Puerta& p) {
  float d = medirDistanciaCm(p.pinTrig, p.pinEcho);
  p.ultimaDistancia = d;
  if (d < 0) return 0;
  float h = p.alturaSensor - d;
  return h > 0 ? h : 0;
}

float mediana(float* v, int n) {
  for (int i = 1; i < n; i++) {               // ordenamiento por inserción
    float x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[n / 2];
}

float medirAlturaMediana(Puerta& p, int n) {
  float v[9];
  n = min(n, 9);
  for (int i = 0; i < n; i++) { v[i] = medirAltura(p); esperarActivo(30); }
  return mediana(v, n);
}

// Mide la distancia al piso al arrancar (¡no debe haber nada debajo!)
void calibrarSensor(Puerta& p) {
  float v[7]; int validas = 0;
  for (int i = 0; i < 7; i++) {
    float d = medirDistanciaCm(p.pinTrig, p.pinEcho);
    if (d > 2 && d < 200) v[validas++] = d;
    esperarActivo(60);
  }
  if (validas >= 4) p.alturaSensor = mediana(v, validas);
  else              p.alturaSensor = ALTURA_SENSOR_DEFECTO_CM;
  logEvento("Calibracion %s: %.1f cm%s", p.nombre, p.alturaSensor, validas >= 4 ? "" : " (defecto)");
}

// =====================================================================
//                               CÁMARA
// =====================================================================
bool iniciarCamara() {
  camera_config_t c = {};
  c.pin_pwdn     = -1;
  c.pin_reset    = -1;
  c.pin_xclk     = CAM_PIN_XCLK;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_d7 = CAM_PIN_D7;  c.pin_d6 = CAM_PIN_D6;
  c.pin_d5 = CAM_PIN_D5;  c.pin_d4 = CAM_PIN_D4;
  c.pin_d3 = CAM_PIN_D3;  c.pin_d2 = CAM_PIN_D2;
  c.pin_d1 = CAM_PIN_D1;  c.pin_d0 = CAM_PIN_D0;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href  = CAM_PIN_HREF;
  c.pin_pclk  = CAM_PIN_PCLK;
  c.xclk_freq_hz = CAM_XCLK_HZ;
  c.ledc_timer   = LEDC_TIMER_0;        // los servos usan otro timer
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_RGB565;    // la OV7670 no genera JPEG
  c.frame_size   = FRAMESIZE_QQVGA;     // 160x120 (cabe en RAM sin PSRAM)
  c.jpeg_quality = 12;
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.sccb_i2c_port = 1;                  // la OLED usa el I2C 0

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camara: error 0x%x (el sistema sigue funcionando sin fotos)\n", err);
    return false;
  }
  corregirVentanaOV7670();
  return true;
}

// Corrección del driver esp32-camera para la OV7670 en QQVGA (160x120).
// El driver recibe cada cuadro en 5 bloques de 24 líneas. Con la ventana
// que pone el driver (filas 12..490) salen 119 líneas, y aun con 120 los
// últimos bytes se quedan en la FIFO del ESP32 hasta que llegan más datos:
// el último bloque nunca se completa y cada cuadro llega con 30720 bytes en
// vez de 38400 ("cam_hal: FB-SIZE: 30720 != 38400"). Se deja la ventana en
// 10..494: 120 líneas más 1 extra que empuja esos bytes. La línea extra no
// se guarda (el driver solo copia 38400 bytes). Probado con prueba_camara:
// 0 líneas extra falla, 1 línea extra funciona.
extern "C" uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
extern "C" int     SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);

void corregirVentanaOV7670() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s || s->id.PID != OV7670_PID) return;
  const uint16_t vInicio = 10, vFin = 494;   // 480 filas + 4 (= 1 línea extra)
  SCCB_Write(s->slv_addr, 0x19, vInicio >> 2);                          // VSTRT
  SCCB_Write(s->slv_addr, 0x1A, vFin >> 2);                             // VSTOP
  SCCB_Write(s->slv_addr, 0x03, ((vFin & 3) << 2) | (vInicio & 3));     // VREF
  Serial.printf("Camara: ventana vertical corregida (VSTRT=0x%02X VSTOP=0x%02X VREF=0x%02X)\n",
                SCCB_Read(s->slv_addr, 0x19), SCCB_Read(s->slv_addr, 0x1A), SCCB_Read(s->slv_addr, 0x03));
  aplicarDivisorReloj(camDivisorReloj);
}

// Escribe el divisor del reloj interno (CLKRC, registro 0x11) de la OV7670
void aplicarDivisorReloj(uint8_t n) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s || s->id.PID != OV7670_PID) return;
  camDivisorReloj = n & 0x3F;
  SCCB_Write(s->slv_addr, 0x11, camDivisorReloj);
  Serial.printf("Camara: divisor de reloj %u (CLKRC=0x%02X). PLK = XLK / %u\n",
                camDivisorReloj, SCCB_Read(s->slv_addr, 0x11), camDivisorReloj + 1);
}

String rutaFoto(uint32_t visita) { return "/f" + String(visita) + ".jpg"; }

// El codificador JPEG llama a esta función con cada trozo que genera y aquí
// se escribe directo en el archivo. Se usa frame2jpg_cb() y no frame2jpg()
// porque frame2jpg() pide un bloque de 128 KB de una vez, que en un ESP32
// sin PSRAM (y con WiFi) nunca está libre: la foto fallaba siempre.
struct SalidaJpg { File* f; size_t total; };
size_t escribirTrozoJpg(void* arg, size_t index, const void* data, size_t len) {
  SalidaJpg* s = (SalidaJpg*)arg;
  size_t n = s->f->write((const uint8_t*)data, len);
  s->total += n;
  return n;
}

// Razón de la última foto fallida (el dashboard la muestra al pulsar "Probar cámara")
char errorFoto[96] = "";

bool capturarJPEG(const String& ruta) {
  if (!camaraOK) {
    snprintf(errorFoto, sizeof(errorFoto), "la camara no inicio (revisa SCL/SDA, 3V3, GND y XLK)");
    return false;
  }
  camera_fb_t* fb = esp_camera_fb_get();          // descarta el cuadro viejo
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) {
    snprintf(errorFoto, sizeof(errorFoto), "no llego ningun cuadro completo de la camara");
    Serial.println(F("Camara: no llego ningun cuadro. Escribe 'cam' para revisar los cables."));
    return false;
  }
  if (CAM_INVERTIR_BYTES && fb->format == PIXFORMAT_RGB565) {
    for (size_t i = 0; i + 1 < fb->len; i += 2) {
      uint8_t b = fb->buf[i]; fb->buf[i] = fb->buf[i + 1]; fb->buf[i + 1] = b;
    }
  }
  // OJO: un File cerrado vale false, así que el resultado se guarda ANTES de close()
  bool guardada = false;
  File f = LittleFS.open(ruta, "w");
  if (!f) {
    snprintf(errorFoto, sizeof(errorFoto), "no se pudo crear el archivo en LittleFS");
  } else {
    SalidaJpg salida = { &f, 0 };
    bool ok = frame2jpg_cb(fb, CALIDAD_JPEG, escribirTrozoJpg, &salida);
    guardada = ok && salida.total > 0;
    f.close();
    if (!guardada)
      snprintf(errorFoto, sizeof(errorFoto), "fallo la conversion a JPEG (%u bytes escritos, memoria libre %u, bloque mayor %u)",
               (unsigned)salida.total, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  }
  esp_camera_fb_return(fb);
  if (!guardada) {
    LittleFS.remove(ruta);
    Serial.printf("Camara: %s\n", errorFoto);
  }
  return guardada;
}

// Parte del comando 'cam': con el contador de pulsos (PCNT) del ESP32 cuenta,
// durante 1 s, cuántos pulsos de PLK llegan mientras HS está en alto (= bytes
// que manda la cámara) y cuántas líneas (pulsos de HS). En 160x120 RGB565
// deben salir 320 bytes por línea, 120 líneas y 38400 bytes por cuadro.
// VS NO se conecta al contador porque el driver de la cámara usa su interrupción.
void medirBytesCamara() {
  pcnt_unit_handle_t uBytes = nullptr, uLineas = nullptr;
  pcnt_channel_handle_t chBytes = nullptr, chLineas = nullptr;
  pcnt_unit_config_t uc = {};
  uc.low_limit  = -30000;
  uc.high_limit =  30000;
  uc.flags.accum_count = 1;           // sigue contando más allá de 30000

  bool ok = pcnt_new_unit(&uc, &uBytes) == ESP_OK &&
            pcnt_new_unit(&uc, &uLineas) == ESP_OK;
  if (ok) {
    pcnt_chan_config_t cb = {};
    cb.edge_gpio_num  = CAM_PIN_PCLK;
    cb.level_gpio_num = CAM_PIN_HREF;
    pcnt_chan_config_t cl = {};
    cl.edge_gpio_num  = CAM_PIN_HREF;
    cl.level_gpio_num = -1;
    ok = pcnt_new_channel(uBytes, &cb, &chBytes) == ESP_OK &&
         pcnt_new_channel(uLineas, &cl, &chLineas) == ESP_OK;
  }
  if (ok) {
    // Bytes: cuenta flancos de subida de PLK solo mientras HS = 1
    pcnt_channel_set_edge_action(chBytes, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(chBytes, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD);
    // Líneas: cuenta flancos de subida de HS
    pcnt_channel_set_edge_action(chLineas, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_unit_handle_t us[2] = { uBytes, uLineas };
    for (auto u : us) {
      pcnt_unit_add_watch_point(u, uc.high_limit);
      pcnt_unit_enable(u);
      pcnt_unit_clear_count(u);
    }
  }
  if (!ok) {
    Serial.println(F("Medicion con contador: no se pudo iniciar el PCNT."));
  } else {
    // Cuadros: se cuentan mirando VS (igual que arriba)
    uint32_t cuadros = 0, k = 0;
    int vsAnt = gpio_get_level((gpio_num_t)CAM_PIN_VSYNC);
    pcnt_unit_start(uBytes);
    pcnt_unit_start(uLineas);
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      int vs = gpio_get_level((gpio_num_t)CAM_PIN_VSYNC);
      if (vs && !vsAnt) cuadros++;
      vsAnt = vs;
      if ((++k & 0x3FF) == 0) yield();
    }
    pcnt_unit_stop(uBytes);
    pcnt_unit_stop(uLineas);
    int bytes = 0, lineas = 0;
    pcnt_unit_get_count(uBytes, &bytes);
    pcnt_unit_get_count(uLineas, &lineas);

    Serial.println(F("Medicion con contador (1 s):"));
    Serial.printf("  Cuadros: %lu   Lineas: %d   Bytes: %d\n", (unsigned long)cuadros, lineas, bytes);
    if (cuadros > 0 && lineas > 0) {
      float bLinea  = (float)bytes / lineas;
      float lCuadro = (float)lineas / cuadros;
      float bCuadro = (float)bytes / cuadros;
      Serial.printf("  Bytes por linea:   %.1f  (deben ser 320)\n", bLinea);
      Serial.printf("  Lineas por cuadro: %.1f  (deben ser 120)\n", lCuadro);
      Serial.printf("  Bytes por cuadro:  %.0f  (deben ser 38400)\n", bCuadro);
      if (bCuadro > 38400 * 0.99f && bCuadro < 38400 * 1.01f)
        Serial.println(F("=> La camara manda la imagen completa (160x120). Mira abajo si la captura sale OK."));
      else if (bLinea > 320 * 0.99f && bLinea < 320 * 1.01f)
        Serial.println(F("=> Las lineas estan completas pero llegan menos lineas por cuadro de las esperadas."));
      else
        Serial.println(F("=> Las lineas llegan cortas. Si el numero es estable (ej. 256), es configuracion; si varia, es ruido en PLK."));
    } else {
      Serial.println(F("=> No se pudieron contar cuadros o lineas."));
    }
  }

  if (chBytes)  pcnt_del_channel(chBytes);
  if (chLineas) pcnt_del_channel(chLineas);
  pcnt_unit_handle_t us[2] = { uBytes, uLineas };
  for (auto u : us) {
    if (!u) continue;
    pcnt_unit_disable(u);   // devuelve error si no estaba habilitada; no importa
    pcnt_del_unit(u);
  }
  gpio_pullup_dis((gpio_num_t)CAM_PIN_PCLK);   // el PCNT le activa la pull-up
}

// Comando 'cam': durante 1 s mira si cada señal de la cámara cambia.
// Una señal "fija" casi siempre es un cable suelto, mal puesto o demasiado largo.
void diagnosticarCamara() {
  if (!camaraOK) {
    Serial.println(F("Camara: no arranco (no responde por SCL/SDA). Revisa 3V3, GND, PWDN a GND, RET a 3V3, XLK y las pull-ups."));
    return;
  }
  const uint8_t pines[] = { CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK,
                            CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2, CAM_PIN_D3,
                            CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7 };
  const char* nombres[] = { "VS ", "HS ", "PLK", "D0 ", "D1 ", "D2 ", "D3 ",
                            "D4 ", "D5 ", "D6 ", "D7 " };
  const int N = sizeof(pines);
  bool vio0[N] = {}, vio1[N] = {};
  uint32_t cuadros = 0, lineas = 0;
  int vsAnt = gpio_get_level((gpio_num_t)CAM_PIN_VSYNC);
  int hsAnt = gpio_get_level((gpio_num_t)CAM_PIN_HREF);
  uint32_t t0 = millis(), k = 0;
  while (millis() - t0 < 1000) {
    for (int i = 0; i < N; i++) {
      if (gpio_get_level((gpio_num_t)pines[i])) vio1[i] = true; else vio0[i] = true;
    }
    int vs = gpio_get_level((gpio_num_t)CAM_PIN_VSYNC);
    int hs = gpio_get_level((gpio_num_t)CAM_PIN_HREF);
    if (vs && !vsAnt) cuadros++;
    if (hs && !hsAnt) lineas++;
    vsAnt = vs; hsAnt = hs;
    if ((++k & 0x3FF) == 0) yield();
  }

  Serial.printf("Camara: XLK a %lu MHz. Senales durante 1 s:\n", (unsigned long)(CAM_XCLK_HZ / 1000000));
  for (int i = 0; i < N; i++) {
    const char* estado = (vio0[i] && vio1[i]) ? "cambia (bien)"
                       : vio1[i] ? "FIJA en 1" : "FIJA en 0";
    Serial.printf("  %s GPIO %2d: %s\n", nombres[i], pines[i], estado);
  }
  Serial.printf("  Cuadros por segundo (VS): %lu   Lineas contadas (HS): %lu\n",
                (unsigned long)cuadros, (unsigned long)lineas);

  bool vsOK  = vio0[0] && vio1[0], hsOK = vio0[1] && vio1[1], plkOK = vio0[2] && vio1[2];
  if (!vsOK && !hsOK && !plkOK)
    Serial.println(F("=> No sale nada de la camara: revisa XLK (GPIO 32), su alimentacion y PWDN a GND."));
  else if (!plkOK)
    Serial.println(F("=> Falla PLK (GPIO 33): cable suelto, mal puesto o muy largo."));
  else if (!vsOK || !hsOK)
    Serial.println(F("=> Falla VS (34) o HS (35): revisa ese cable y que no esten intercambiados."));
  else {
    bool datosFijos = false;
    for (int i = 3; i < N; i++) if (!(vio0[i] && vio1[i])) datosFijos = true;
    if (datosFijos)
      Serial.println(F("=> Algun D0-D7 no cambia. Apunta la camara a algo con luz y repite; si sigue fijo, revisa ese cable."));
    else
      Serial.println(F("=> Todas las senales llegan. Si la foto falla, es ruido: acorta PLK y XLK o baja CAM_XCLK_HZ."));
  }

  medirBytesCamara();

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    Serial.printf("Prueba de captura: OK (%ux%u, %u bytes)\n", fb->width, fb->height, (unsigned)fb->len);
    esp_camera_fb_return(fb);
  } else {
    Serial.println(F("Prueba de captura: FALLA (no llego ningun cuadro completo)"));
    Serial.println(F("=> Si la camara manda la imagen completa pero esto falla, el ESP32 pierde datos al recibirla: mira el numero de FB-SIZE."));
  }
}

bool capturarFoto(uint32_t visita) {
  if (visita > (uint32_t)MAX_FOTOS) LittleFS.remove(rutaFoto(visita - MAX_FOTOS));
  return capturarJPEG(rutaFoto(visita));
}

void borrarFotosViejas() {
  String nombres[64]; int n = 0;
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f && n < 64) {
    String p = f.path();
    f.close();
    if (p.startsWith("/f") && p.endsWith(".jpg")) nombres[n++] = p;
    f = root.openNextFile();
  }
  for (int i = 0; i < n; i++) LittleFS.remove(nombres[i]);
}

// =====================================================================
//                            PANTALLA OLED
// =====================================================================
char     msgL1[12] = "", msgL2[12] = "", msgL3[22] = "";
uint32_t tMensaje = 0;
const uint32_t DURACION_MENSAJE_MS = 2500;
uint32_t tRefrescoOled = 0;

void mostrarMensaje(const char* l1, const char* l2, const char* l3) {
  strncpy(msgL1, l1, sizeof(msgL1) - 1);
  strncpy(msgL2, l2, sizeof(msgL2) - 1);
  strncpy(msgL3, l3, sizeof(msgL3) - 1);
  tMensaje = millis();
  if (tMensaje == 0) tMensaje = 1;
}

void contarDentro(int& h, int& r) {
  h = r = 0;
  for (auto& w : trabajadores) if (w.dentro) (w.tipo == HUMANO ? h : r)++;
}

void oledInicio(const char* linea) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.println("CONTROL DE ACCESO");
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  oled.setCursor(0, 20); oled.println(linea);
  oled.display();
}

void actualizarOLED() {
  if (!oledOK || millis() - tRefrescoOled < 250) return;
  tRefrescoOled = millis();
  uint32_t now = millis();
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // --- Pantalla de emergencia ---
  if (emergencia) {
    bool inv = (now / 500) % 2;
    if (inv) oled.fillRect(0, 0, 128, 64, SSD1306_WHITE);
    oled.setTextColor(inv ? SSD1306_BLACK : SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(4, 6);   oled.print("EMERGENCIA");
    oled.setTextSize(1);
    oled.setCursor(10, 32); oled.print("Puertas ABIERTAS");
    oled.setCursor(13, 46); oled.print("RESET: boton BOOT");
    oled.setCursor(4, 56);  oled.print("o mantener boton 3s");
    oled.display();
    return;
  }

  // --- Mensaje temporal (entrada / salida / aviso) ---
  if (tMensaje && now - tMensaje < DURACION_MENSAJE_MS) {
    oled.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor((128 - strlen(msgL1) * 12) / 2, 6);  oled.print(msgL1);
    oled.setCursor((128 - strlen(msgL2) * 12) / 2, 26); oled.print(msgL2);
    oled.setTextSize(1);
    oled.setCursor((128 - strlen(msgL3) * 6) / 2, 50);  oled.print(msgL3);
    oled.display();
    return;
  }

  int h, r; contarDentro(h, r);
  bool paginaLista = (now / 4000) % 2 && (h + r) > 0;

  // Encabezado
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(paginaLista ? "EN PLANTA" : "RESUMEN PLANTA");
  oled.setCursor(98, 0);
  oled.printf("%2d/%d", h + r, MAX_TRABAJADORES);
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (!paginaLista) {
    oled.setCursor(0, 13); oled.printf("Humanos: %d", h);
    oled.setCursor(0, 23); oled.printf("Robots : %d", r);
    oled.setCursor(0, 35);
    oled.printf("Entrada:%s", puertas[0].estado == P_ABIERTA ? "ABIERTA" : "cerrada");
    oled.setCursor(0, 45);
    oled.printf("Salida :%s", puertas[1].estado == P_ABIERTA ? "ABIERTA" : "cerrada");
    oled.setCursor(0, 56);
    oled.print(WiFi.softAPIP());
    if (!camaraOK) { oled.setCursor(98, 56); oled.print("!CAM"); }
  } else {
    int y = 12;
    for (auto& w : trabajadores) {
      if (!w.dentro) continue;
      if (y > 56) break;
      uint32_t s = (now - w.tEntrada) / 1000;
      oled.setCursor(0, y);
      oled.printf("%s %-6s %3lus%s", w.id, nombreTipo(w.tipo), (unsigned long)s, w.alerta ? " !" : "");
      y += 9;
    }
  }
  oled.display();
}

// =====================================================================
//                   LÓGICA DE TRABAJADORES / HISTORIAL
// =====================================================================
void agregarHistorial(const Trabajador& w) {
  for (int i = MAX_HISTORIAL - 1; i > 0; i--) historial[i] = historial[i - 1];
  Registro& r = historial[0];
  r.visita = w.visita; strcpy(r.id, w.id); r.tipo = w.tipo;
  r.tEntrada = w.tEntrada; r.tSalida = 0; r.dentro = true; r.foto = w.foto;
  if (nHistorial < MAX_HISTORIAL) nHistorial++;
}

void cerrarHistorial(uint32_t visita, uint32_t tSalida) {
  for (int i = 0; i < nHistorial; i++)
    if (historial[i].visita == visita && historial[i].dentro) {
      historial[i].dentro = false;
      historial[i].tSalida = tSalida;
    }
}

int buscarSlotLibre() {
  for (int i = 0; i < MAX_TRABAJADORES; i++) if (!trabajadores[i].dentro) return i;
  return -1;
}

// Trabajador marcado (desde el dashboard o el Monitor Serie) como el
// próximo en salir. Devuelve su posición, o -1 si no hay marca vigente.
int slotMarcadoSalida() {
  if (!visitaMarcadaSalida) return -1;
  if (millis() - tMarcaSalida >= VIGENCIA_MARCA_SALIDA_MS) { visitaMarcadaSalida = 0; return -1; }
  for (int i = 0; i < MAX_TRABAJADORES; i++)
    if (trabajadores[i].dentro && trabajadores[i].visita == visitaMarcadaSalida) return i;
  visitaMarcadaSalida = 0;
  return -1;
}

bool marcarSalida(int slot) {
  if (slot < 0 || slot >= MAX_TRABAJADORES || !trabajadores[slot].dentro) return false;
  visitaMarcadaSalida = trabajadores[slot].visita;
  tMarcaSalida = millis();
  mostrarMensaje("SALIDA", trabajadores[slot].id, "Marcado para salir");
  logEvento("%s marcado como proxima salida", trabajadores[slot].id);
  return true;
}

// Sin lector de tarjetas el sensor no sabe QUIÉN sale. Si alguien fue
// marcado como próxima salida, se usa esa marca. Si no, se asume que sale
// el trabajador del mismo tipo que lleva más tiempo dentro (FIFO por tipo).
int buscarParaSalida(Tipo t) {
  int mejor = -1, cualquiera = -1;
  for (int i = 0; i < MAX_TRABAJADORES; i++) {
    Trabajador& w = trabajadores[i];
    if (!w.dentro) continue;
    if (cualquiera < 0 || (int32_t)(w.tEntrada - trabajadores[cualquiera].tEntrada) < 0) cualquiera = i;
    if (w.tipo == t && (mejor < 0 || (int32_t)(w.tEntrada - trabajadores[mejor].tEntrada) < 0)) mejor = i;
  }
  return mejor >= 0 ? mejor : cualquiera;
}

void registrarEntrada(Tipo t, Puerta& p) {
  int slot = buscarSlotLibre();
  if (slot < 0) {
    logEvento("Planta llena (%d): acceso denegado", MAX_TRABAJADORES);
    SONAR(PAT_ERROR);
    mostrarMensaje("PLANTA", "LLENA", "Acceso denegado");
    p.estado = P_REARME; p.tEstado = millis(); p.conteo = 0;
    return;
  }
  Trabajador& w = trabajadores[slot];
  w.visita = ++contadorVisitas;
  w.tipo   = t;
  snprintf(w.id, sizeof(w.id), "T%02d", slot + 1);
  w.foto   = capturarFoto(w.visita);    // primero la foto...
  w.tEntrada = millis();                // ...luego inicia el contador
  w.alerta = false;
  w.dentro = true;
  abrirPuerta(p);
  agregarHistorial(w);

  if (t == ROBOT) SONAR(PAT_ROBOT); else if (BEEP_HUMANOS) SONAR(PAT_HUMANO);
  mostrarMensaje("ENTRA", w.id, nombreTipo(t));
  logEvento("Entrada %s (%s)%s", w.id, nombreTipo(t), w.foto ? "" : " sin foto");
}

void registrarSalida(Tipo t, Puerta& p) {
  abrirPuerta(p);                       // por seguridad siempre se deja salir
  if (t == ROBOT) SONAR(PAT_ROBOT); else if (BEEP_HUMANOS) SONAR(PAT_HUMANO);

  int slot = slotMarcadoSalida();
  visitaMarcadaSalida = 0;              // la marca se usa una sola vez
  if (slot >= 0 && trabajadores[slot].tipo != t)
    logEvento("Aviso: %s es %s, el sensor vio %s", trabajadores[slot].id,
              nombreTipo(trabajadores[slot].tipo), nombreTipo(t));
  if (slot < 0) slot = buscarParaSalida(t);
  if (slot < 0) {
    logEvento("Salida de %s sin registro de entrada", nombreTipo(t));
    mostrarMensaje("SALE", "?", "Sin registro");
    return;
  }
  Trabajador& w = trabajadores[slot];
  uint32_t ahora = millis();
  uint32_t dur = (ahora - w.tEntrada) / 1000;
  w.dentro = false;
  cerrarHistorial(w.visita, ahora);
  char l3[22];
  snprintf(l3, sizeof(l3), "%s %lus", nombreTipo(w.tipo), (unsigned long)dur);
  mostrarMensaje("SALE", w.id, l3);
  logEvento("Salida %s (%s) tras %lu s", w.id, nombreTipo(w.tipo), (unsigned long)dur);
}

// Clasifica por altura. Devuelve false si cae en la zona muerta (entre
// ROBOT_MAX_CM y HUMANO_MIN_CM) y no registra nada. Con forzar = true (en
// emergencia, para no perder a nadie) usa el punto medio de la zona muerta.
bool procesarLlegada(Puerta& p, bool forzar) {
  float h = medirAlturaMediana(p, 5);
  if (h < ALTURA_MIN_PRESENCIA_CM) h = p.ultimaAltura;      // respaldo
  Tipo t;
  if      (h >= HUMANO_MIN_CM) t = HUMANO;
  else if (h <= ROBOT_MAX_CM)  t = ROBOT;
  else if (forzar)             t = (h >= (ROBOT_MAX_CM + HUMANO_MIN_CM) / 2) ? HUMANO : ROBOT;
  else {
    Serial.printf("  %s: altura %.1f cm -> DUDOSA (robot <= %.1f, humano >= %.1f)\n",
                  p.nombre, h, ROBOT_MAX_CM, HUMANO_MIN_CM);
    char l2[12], l3[22];
    snprintf(l2, sizeof(l2), "%.1f cm", h);
    snprintf(l3, sizeof(l3), "Robot<%.1f Humano>%.1f", ROBOT_MAX_CM, HUMANO_MIN_CM);
    mostrarMensaje("DUDOSO", l2, l3);
    return false;
  }
  Serial.printf("  %s: altura %.1f cm -> %s\n", p.nombre, h, nombreTipo(t));
  if (p.esEntrada) registrarEntrada(t, p);
  else             registrarSalida(t, p);
  return true;
}

void revisarTiempos() {
  uint32_t now = millis();
  for (auto& w : trabajadores) {
    if (w.dentro && !w.alerta && now - w.tEntrada > TIEMPO_MAX_PERMANENCIA_MS) {
      w.alerta = true;
      logEvento("%s supera %lu s en planta", w.id, (unsigned long)(TIEMPO_MAX_PERMANENCIA_MS / 1000));
      SONAR(PAT_ALERTA);
      mostrarMensaje("ALERTA", w.id, "Tiempo excedido");
    }
  }
}

// =====================================================================
//                       MÁQUINA DE ESTADOS DE PUERTAS
// =====================================================================
void actualizarPuerta(Puerta& p) {
  uint32_t now = millis();
  float h = medirAltura(p);
  bool hay = h >= ALTURA_MIN_PRESENCIA_CM;
  p.ultimaAltura = h;

  switch (p.estado) {
    case P_CERRADA:                       // esperando a alguien
      if (hay) {
        if (now - p.tDudosa < REINTENTO_DUDOSA_MS) break;   // espera antes de volver a medir
        if (++p.conteo >= LECTURAS_CONFIRMACION) {
          p.conteo = 0;
          if (!procesarLlegada(p, false)) p.tDudosa = millis();
        }
      } else { p.conteo = 0; p.tDudosa = 0; }
      break;

    case P_ABIERTA: {
      if (hay) p.tUltimaPresencia = now;
      uint32_t abierta = now - p.tEstado;
      if (p.remota) {
        if (abierta >= APERTURA_REMOTA_MS && !hay) cerrarPuerta(p);
      } else if ((abierta >= PUERTA_ABIERTA_MIN_MS && now - p.tUltimaPresencia >= PUERTA_ESPERA_LIBRE_MS)
                 || abierta >= PUERTA_ABIERTA_MAX_MS) {
        cerrarPuerta(p);
      }
      break;
    }

    case P_REARME:                        // cerrada, espera zona libre
      if (now - p.tEstado < 700) break;
      if (!hay) { if (++p.conteo >= 3) { p.conteo = 0; p.estado = P_CERRADA; } }
      else p.conteo = 0;
      break;
  }
}

// En emergencia las puertas quedan abiertas, pero se sigue contando quién
// pasa: así el dashboard muestra quién sigue dentro durante la evacuación.
void vigilarPuertaEmergencia(Puerta& p) {
  float h = medirAltura(p);
  bool hay = h >= ALTURA_MIN_PRESENCIA_CM;
  p.ultimaAltura = h;
  if (hay == p.ocupada) { p.conteo = 0; return; }
  if (++p.conteo < LECTURAS_CONFIRMACION) return;
  p.conteo = 0;
  p.ocupada = hay;
  if (hay) procesarLlegada(p, true);      // registra; la puerta ya está abierta
}

void actualizarPuertas() {
  if (millis() - tUltimaMedicion < PERIODO_MEDICION_MS) return;
  tUltimaMedicion = millis();
  puertaTurno ^= 1;                       // alterna para no mezclar ecos
  if (emergencia) vigilarPuertaEmergencia(puertas[puertaTurno]);
  else            actualizarPuerta(puertas[puertaTurno]);
}

// =====================================================================
//                       EMERGENCIA Y RESET
// =====================================================================
void activarEmergencia(const char* origen) {
  if (emergencia) return;
  emergencia = true;
  tEmergencia = millis();
  for (auto& p : puertas) {
    // Si la puerta estaba abierta o en rearme, quien está debajo ya se contó
    p.ocupada = (p.estado != P_CERRADA);
    abrirPuerta(p);
  }
  SONAR(PAT_EMERGENCIA);
  logEvento("EMERGENCIA activada (%s)", origen);
}

void resetSistema(const char* origen) {
  emergencia = false;
  visitaMarcadaSalida = 0;
  uint32_t now = millis();
  for (auto& w : trabajadores) {
    if (w.dentro) cerrarHistorial(w.visita, now);
    w.dentro = false;
    w.alerta = false;
  }
  for (auto& p : puertas) cerrarPuerta(p);
  SONAR(PAT_RESET);
  mostrarMensaje("RESET", "OK", "Contadores en cero");
  logEvento("Reset del sistema (%s)", origen);
}

// =====================================================================
//                               BOTONES
// =====================================================================
// Devuelve: 0 nada, 1 pulsación corta (al soltar), 2 pulsación larga
int leerBoton(Boton& b) {
  if (b.pin < 0) return 0;
  bool lectura = digitalRead(b.pin) == LOW;   // a GND con pull-up
  uint32_t now = millis();
  if (lectura != b.lecturaPrev) { b.lecturaPrev = lectura; b.tCambio = now; }
  if (now - b.tCambio < ANTIRREBOTE_MS) return 0;

  if (lectura && !b.presionado) { b.presionado = true; b.tPresion = now; b.largoHecho = false; }
  else if (!lectura && b.presionado) {
    b.presionado = false;
    if (!b.largoHecho) return 1;
  }
  if (b.presionado && !b.largoHecho && now - b.tPresion >= PULSACION_LARGA_MS) {
    b.largoHecho = true;
    return 2;
  }
  return 0;
}

void leerBotones() {
  int e = leerBoton(boton);
  if (e == 1) activarEmergencia("boton");
  if (e == 2) resetSistema("boton");
  if (leerBoton(botonReset) != 0) resetSistema("boton BOOT");
}

// =====================================================================
//                         SERVIDOR WEB / API
// =====================================================================
void enviarJSON(int code, const String& s) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", s);
}

void agregarTexto(String& j, const char* s) {     // texto con escape JSON
  j += '"';
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') j += '\\';
    j += *s;
  }
  j += '"';
}

void handleEstado() {
  uint32_t now = millis();
  int h, r; contarDentro(h, r);
  String j; j.reserve(5000);
  j += "{\"ahora\":"; j += now;
  j += ",\"boot\":"; j += bootId;
  j += ",\"emergencia\":"; j += emergencia ? "true" : "false";
  j += ",\"camara\":"; j += camaraOK ? "true" : "false";
  j += ",\"oled\":true";
  j += ",\"max\":"; j += MAX_TRABAJADORES;
  j += ",\"tmax\":"; j += TIEMPO_MAX_PERMANENCIA_MS / 1000;
  j += ",\"humanos\":"; j += h;
  j += ",\"robots\":"; j += r;
  int marcado = slotMarcadoSalida();
  j += ",\"proxSalida\":"; j += marcado >= 0 ? trabajadores[marcado].visita : 0;

  j += ",\"puertas\":[";
  for (int i = 0; i < 2; i++) {
    Puerta& p = puertas[i];
    if (i) j += ',';
    j += "{\"nombre\":\""; j += p.nombre;
    j += "\",\"abierta\":"; j += (p.estado == P_ABIERTA || emergencia) ? "true" : "false";
    j += ",\"altura\":"; j += String(p.ultimaAltura, 1);
    j += ",\"piso\":"; j += String(p.alturaSensor, 1);
    j += '}';
  }
  j += ']';

  j += ",\"dentro\":[";
  bool primero = true;
  for (auto& w : trabajadores) {
    if (!w.dentro) continue;
    if (!primero) j += ',';
    primero = false;
    j += "{\"id\":\""; j += w.id;
    j += "\",\"tipo\":\""; j += w.tipo == HUMANO ? 'H' : 'R';
    j += "\",\"visita\":"; j += w.visita;
    j += ",\"t\":"; j += w.tEntrada;
    j += ",\"foto\":"; j += w.foto ? "true" : "false";
    j += ",\"alerta\":"; j += w.alerta ? "true" : "false";
    j += '}';
  }
  j += ']';

  j += ",\"historial\":[";
  for (int i = 0; i < nHistorial; i++) {
    Registro& rg = historial[i];
    if (i) j += ',';
    j += "{\"id\":\""; j += rg.id;
    j += "\",\"tipo\":\""; j += rg.tipo == HUMANO ? 'H' : 'R';
    j += "\",\"visita\":"; j += rg.visita;
    j += ",\"te\":"; j += rg.tEntrada;
    j += ",\"ts\":"; j += rg.tSalida;
    j += ",\"dentro\":"; j += rg.dentro ? "true" : "false";
    j += ",\"foto\":"; j += rg.foto ? "true" : "false";
    j += '}';
  }
  j += ']';

  j += ",\"eventos\":[";
  for (int i = 0; i < nEventos; i++) {
    if (i) j += ',';
    j += "{\"t\":"; j += eventos[i].t;
    j += ",\"txt\":"; agregarTexto(j, eventos[i].txt);
    j += '}';
  }
  j += "]}";
  enviarJSON(200, j);
}

void handleFoto() {
  uint32_t v = (uint32_t)server.arg("v").toInt();   // solo números: evita rutas raras
  String ruta = rutaFoto(v);
  if (v == 0 || !LittleFS.exists(ruta)) { server.send(404, "text/plain", "sin foto"); return; }
  File f = LittleFS.open(ruta, "r");
  server.sendHeader("Cache-Control", "max-age=86400");
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleFotoPrueba() {
  if (!capturarJPEG("/prueba.jpg")) { server.send(500, "text/plain", errorFoto); return; }
  File f = LittleFS.open("/prueba.jpg", "r");
  if (!f) { server.send(500, "text/plain", "la foto se guardo pero no se pudo leer"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleAbrir() {
  String q = server.arg("puerta");
  int idx = q == "entrada" ? 0 : q == "salida" ? 1 : -1;
  if (idx < 0) { enviarJSON(400, "{\"ok\":false,\"error\":\"puerta invalida\"}"); return; }
  if (emergencia) { enviarJSON(409, "{\"ok\":false,\"error\":\"emergencia activa\"}"); return; }
  abrirPuerta(puertas[idx], true);
  logEvento("Apertura remota: %s", puertas[idx].nombre);
  enviarJSON(200, "{\"ok\":true}");
}

void handleEmergencia() { activarEmergencia("dashboard"); enviarJSON(200, "{\"ok\":true}"); }
void handleReset()      { resetSistema("dashboard");      enviarJSON(200, "{\"ok\":true}"); }

// Recalibra la distancia al piso de los dos sensores (la zona debe estar vacía)
void handleCalibrar() {
  for (auto& p : puertas) calibrarSensor(p);
  enviarJSON(200, "{\"ok\":true}");
}

// ?id=T03 marca a ese trabajador como el próximo en salir; ?id= (vacío) cancela
void handleMarcarSalida() {
  String id = server.arg("id");
  if (id.length() == 0) {
    visitaMarcadaSalida = 0;
    enviarJSON(200, "{\"ok\":true}");
    return;
  }
  for (int i = 0; i < MAX_TRABAJADORES; i++)
    if (trabajadores[i].dentro && id == trabajadores[i].id) {
      marcarSalida(i);
      enviarJSON(200, "{\"ok\":true}");
      return;
    }
  enviarJSON(404, "{\"ok\":false,\"error\":\"ese trabajador no esta dentro\"}");
}

void iniciarWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/api/estado", HTTP_GET, handleEstado);
  server.on("/foto", HTTP_GET, handleFoto);
  server.on("/api/foto-prueba", HTTP_GET, handleFotoPrueba);
  server.on("/api/abrir", HTTP_POST, handleAbrir);
  server.on("/api/emergencia", HTTP_POST, handleEmergencia);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/calibrar", HTTP_POST, handleCalibrar);
  server.on("/api/marcar-salida", HTTP_POST, handleMarcarSalida);
  server.onNotFound([]() { server.send(404, "text/plain", "No encontrado"); });
  server.begin();
}

void iniciarWiFi() {
  WiFi.mode(STA_SSID[0] ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi AP \"%s\"  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  if (STA_SSID[0]) {
    WiFi.begin(STA_SSID, STA_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("WiFi red \"%s\"  ->  http://%s\n", STA_SSID, WiFi.localIP().toString().c_str());
    else
      Serial.println("No se pudo conectar a la red WiFi (sigue disponible el AP)");
  }
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("Tambien: http://%s.local\n", MDNS_NAME);
  }
}

// =====================================================================
//              MONITOR SERIE (pruebas sin hardware completo)
// =====================================================================
void imprimirAyuda() {
  Serial.println(F("Comandos: eh/er = entra humano/robot | sh/sr = sale humano/robot"));
  Serial.println(F("          em = emergencia | rs = reset | st = estado | d = distancias"));
  Serial.println(F("          cal = recalibrar sensores | p3 = marcar a T03 como proxima salida"));
  Serial.println(F("          cam = revisar las senales y cables de la camara"));
  Serial.println(F("          div N = divisor del reloj de la camara (1 normal, 2-3 mas lento)"));
  Serial.println(F("          dd = encender/apagar la distancia en tiempo real"));
}

void leerSerial() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim(); c.toLowerCase();
  if      (c == "eh") registrarEntrada(HUMANO, puertas[0]);
  else if (c == "er") registrarEntrada(ROBOT,  puertas[0]);
  else if (c == "sh") registrarSalida(HUMANO,  puertas[1]);
  else if (c == "sr") registrarSalida(ROBOT,   puertas[1]);
  else if (c == "em") activarEmergencia("serial");
  else if (c == "rs") resetSistema("serial");
  else if (c == "cal") { for (auto& p : puertas) calibrarSensor(p); }
  else if (c == "cam") diagnosticarCamara();
  else if (c.startsWith("div ")) {
    int n = c.substring(4).toInt();
    if (n < 0 || n > 15) Serial.println(F("Usa un valor entre 0 y 15 (el driver usa 1)"));
    else if (camaraOK) aplicarDivisorReloj(n);
  }
  else if (c.length() >= 2 && c[0] == 'p' && isDigit(c[1])) {
    if (!marcarSalida(c.substring(1).toInt() - 1)) Serial.println(F("Ese trabajador no esta dentro"));
  }
  else if (c == "dd") {
    mostrarDistancias = !mostrarDistancias;
    Serial.printf("Distancia en tiempo real: %s\n", mostrarDistancias ? "ENCENDIDA" : "apagada");
  }
  else if (c == "d") {
    for (auto& p : puertas) {
      float h = medirAltura(p);
      const char* zona = h < ALTURA_MIN_PRESENCIA_CM ? "vacio"
                       : h <= ROBOT_MAX_CM           ? "ROBOT"
                       : h <  HUMANO_MIN_CM          ? "zona muerta"
                                                     : "HUMANO";
      Serial.printf("%s: piso %.1f cm, altura objeto %.1f cm -> %s\n", p.nombre, p.alturaSensor, h, zona);
    }
  } else if (c == "st") {
    int h, r; contarDentro(h, r);
    Serial.printf("Dentro: %d humanos, %d robots | emergencia: %s | camara: %s\n",
                  h, r, emergencia ? "SI" : "no", camaraOK ? "OK" : "FALLA");
    for (auto& w : trabajadores)
      if (w.dentro) Serial.printf("  %s %-6s %lus\n", w.id, nombreTipo(w.tipo), (unsigned long)((millis() - w.tEntrada) / 1000));
  } else imprimirAyuda();
}

// =====================================================================
//                            SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Control de acceso a planta - V1 (OLED + dashboard) ==="));
  bootId = esp_random() & 0xFFFFFF;

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  if (PIN_BOTON_RESET >= 0) pinMode(PIN_BOTON_RESET, INPUT_PULLUP);
  for (auto& p : puertas) {
    pinMode(p.pinTrig, OUTPUT);
    digitalWrite(p.pinTrig, LOW);
    pinMode(p.pinEcho, INPUT);
    iniciarServo(p);
    moverServo(p.pinServo, SERVO_CERRADO_GRADOS);
  }

  // Pantalla
  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOK) Serial.println(F("OLED no encontrada (revisa cableado/direccion)"));
  oledInicio("Iniciando...");

  // Memoria para fotos
  if (!LittleFS.begin(true)) Serial.println(F("Error montando LittleFS"));
  borrarFotosViejas();

  // Cámara (antes del WiFi para tener RAM contigua)
  oledInicio("Iniciando camara...");
  camaraOK = iniciarCamara();
  logEvento(camaraOK ? "Camara OV7670 lista" : "Camara NO detectada");

  // Calibración de los ultrasonidos (zona vacía)
  oledInicio("Calibrando sensores\n(zona despejada)");
  for (auto& p : puertas) calibrarSensor(p);

  oledInicio("Iniciando WiFi...");
  iniciarWiFi();
  iniciarWeb();

  logEvento("Sistema listo");
  imprimirAyuda();
  SONAR(PAT_HUMANO);
}

// Imprime la última distancia medida por cada sensor (no hace mediciones
// extra: usa las que ya hace la máquina de estados de las puertas).
void mostrarDistanciasSerie() {
  if (!mostrarDistancias || millis() - tMostrarDist < PERIODO_MOSTRAR_DIST_MS) return;
  tMostrarDist = millis();
  for (int i = 0; i < 2; i++) {
    Puerta& p = puertas[i];
    if (i) Serial.print(F("  |  "));
    if (p.ultimaDistancia < 0) Serial.printf("%s: sin eco", p.nombre);
    else Serial.printf("%s: %5.1f cm (altura %4.1f)", p.nombre, p.ultimaDistancia, p.ultimaAltura);
  }
  Serial.println();
}

void loop() {
  server.handleClient();
  leerBotones();
  actualizarPuertas();
  mostrarDistanciasSerie();
  revisarTiempos();
  actualizarBuzzer();
  actualizarOLED();
  leerSerial();
}
