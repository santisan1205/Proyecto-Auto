/*
  =====================================================================
   PROYECTO DE AUTOMATIZACIÓN – CONTROL DE ACCESO A PLANTA
   (humanos y robots, 2 puertas, foto, contador de permanencia)

   VERSIÓN 2: SIN PANTALLA – PANEL LOCAL = DASHBOARD WEB
  =====================================================================
   Placa : ESP32 DevKit V1 (ESP32-WROOM-32, 30 o 38 pines, sin PSRAM)
   Core  : "esp32 by Espressif Systems" versión 3.x (probado con 3.3.8)
   Librerías externas: NINGUNA
   (WiFi, WebServer, ESPmDNS, LittleFS y la cámara vienen con el core)

   Requerimientos cubiertos:
     1. Detecta presencia (ultrasonido) y abre la puerta (servo), en
        entrada y salida.
     2. Inicia un contador de permanencia por trabajador.
     3. Identifica HUMANO o ROBOT por la altura medida por el ultrasonido
        (sensor montado arriba de la zona de acceso mirando hacia abajo).
     4. Captura la foto (OV7670) y la asocia a un ID (T01..T12).
     5. Panel de visualización local: el propio ESP32 crea una red WiFi
        y sirve el dashboard (no necesita Internet). También Monitor Serie.
     6. Pulsador: pulsación corta = EMERGENCIA (abre ambas puertas);
        mantener 3 s = RESET (cierra puertas y reinicia contadores).
        Opcional: segundo pulsador de RESET en GPIO21.
   Funcionalidades adicionales:
     1. Dashboard web (WiFi) con ID, foto, tiempo dentro e historial.
     2. Apertura remota de las puertas desde el dashboard.
     3. Señal sonora (buzzer) cada vez que un robot entra o sale.
   Extras: alerta si alguien supera 90 s, autocalibración del
   ultrasonido, comandos por Monitor Serie para simular entradas.
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
#include <stdarg.h>
#include "esp_camera.h"
#include "img_converters.h"
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
#define CAM_PIN_D0      4
#define CAM_PIN_D1      5
#define CAM_PIN_D2     18
#define CAM_PIN_D3     19
#define CAM_PIN_D4     13
#define CAM_PIN_D5     14
#define CAM_PIN_D6     16
#define CAM_PIN_D7     17

// Ultrasonidos HC-SR04 (uno por puerta). ECHO con divisor de voltaje 1k/2k.
#define PIN_TRIG_ENTRADA  23
#define PIN_ECHO_ENTRADA  36   // VP (solo entrada)
#define PIN_TRIG_SALIDA   22
#define PIN_ECHO_SALIDA   39   // VN (solo entrada)

// Servomotores (señal). Alimentarlos con 5V externos, GND común.
#define PIN_SERVO_ENTRADA 25
#define PIN_SERVO_SALIDA  12

// Pulsador (a GND): corto = emergencia, mantener 3 s = reset
#define PIN_BOTON          15
#define PIN_BOTON_RESET    21   // opcional (si no lo conectas no pasa nada)

// Buzzer ACTIVO (HIGH = suena). El LED azul de la placa también parpadea.
#define PIN_BUZZER          2

// ---------------- Parámetros del proceso ----------------
const int      MAX_TRABAJADORES          = 12;
const uint32_t TIEMPO_MAX_PERMANENCIA_MS = 90000;  // 90 s

// Clasificación por altura (en cm). El sensor está arriba mirando al piso:
//   altura del objeto = distancia al piso (calibrada) - distancia medida
const float ALTURA_MIN_PRESENCIA_CM = 2.0;   // más alto que esto = hay alguien
const float UMBRAL_HUMANO_CM        = 8.0;   // >= esto HUMANO, menor ROBOT
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

// Botones
const uint32_t PULSACION_LARGA_MS = 3000;
const uint32_t ANTIRREBOTE_MS     = 30;

// Sonido
const bool     BEEP_HUMANOS            = false; // true = pitido corto también para humanos
const uint32_t DURACION_ALARMA_EMERG_MS = 10000;

// Fotos
const int     MAX_FOTOS     = 40;   // fotos guardadas en memoria (LittleFS)
const uint8_t CALIDAD_JPEG  = 80;

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
};
Puerta puertas[2] = {
  { "Entrada", true,  PIN_TRIG_ENTRADA, PIN_ECHO_ENTRADA, PIN_SERVO_ENTRADA, 4, ALTURA_SENSOR_DEFECTO_CM, P_CERRADA, 0, 0, 0, false, 0 },
  { "Salida",  false, PIN_TRIG_SALIDA,  PIN_ECHO_SALIDA,  PIN_SERVO_SALIDA,  5, ALTURA_SENSOR_DEFECTO_CM, P_CERRADA, 0, 0, 0, false, 0 },
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
uint32_t contadorVisitas = 0;
uint32_t bootId          = 0;
uint32_t tUltimaMedicion = 0;
uint8_t  puertaTurno     = 0;

WebServer server(80);

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
  for (int i = 0; i < n; i++) { v[i] = medirAltura(p); delay(30); }
  return mediana(v, n);
}

// Mide la distancia al piso al arrancar (¡no debe haber nada debajo!)
void calibrarSensor(Puerta& p) {
  float v[7]; int validas = 0;
  for (int i = 0; i < 7; i++) {
    float d = medirDistanciaCm(p.pinTrig, p.pinEcho);
    if (d > 2 && d < 200) v[validas++] = d;
    delay(60);
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
  c.xclk_freq_hz = 10000000;            // 10 MHz va bien con la OV7670
  c.ledc_timer   = LEDC_TIMER_0;        // los servos usan otro timer
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_RGB565;    // la OV7670 no genera JPEG
  c.frame_size   = FRAMESIZE_QQVGA;     // 160x120 (cabe en RAM sin PSRAM)
  c.jpeg_quality = 12;
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.sccb_i2c_port = 1;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camara: error 0x%x (el sistema sigue funcionando sin fotos)\n", err);
    return false;
  }
  return true;
}

String rutaFoto(uint32_t visita) { return "/f" + String(visita) + ".jpg"; }

bool capturarJPEG(const String& ruta) {
  if (!camaraOK) return false;
  camera_fb_t* fb = esp_camera_fb_get();          // descarta el cuadro viejo
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) return false;
  uint8_t* jpg = nullptr; size_t len = 0;
  bool ok = frame2jpg(fb, CALIDAD_JPEG, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) return false;
  File f = LittleFS.open(ruta, "w");
  if (f) { f.write(jpg, len); f.close(); }
  free(jpg);
  return (bool)f;
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
//         MENSAJES LOCALES (sin pantalla: Monitor Serie + dashboard)
// =====================================================================
void mostrarMensaje(const char* l1, const char* l2, const char* l3) {
  Serial.printf("  >> %s %s  %s\n", l1, l2, l3);
}

void contarDentro(int& h, int& r) {
  h = r = 0;
  for (auto& w : trabajadores) if (w.dentro) (w.tipo == HUMANO ? h : r)++;
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

// Sin lector de tarjetas no se sabe QUIÉN sale; se asume que sale el
// trabajador del mismo tipo que lleva más tiempo dentro (FIFO por tipo).
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

  int slot = buscarParaSalida(t);
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

void procesarLlegada(Puerta& p) {
  float h = medirAlturaMediana(p, 5);
  if (h < ALTURA_MIN_PRESENCIA_CM) h = p.ultimaAltura;      // respaldo
  Tipo t = (h >= UMBRAL_HUMANO_CM) ? HUMANO : ROBOT;
  Serial.printf("  %s: altura %.1f cm -> %s\n", p.nombre, h, nombreTipo(t));
  if (p.esEntrada) registrarEntrada(t, p);
  else             registrarSalida(t, p);
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
        if (++p.conteo >= LECTURAS_CONFIRMACION) { p.conteo = 0; procesarLlegada(p); }
      } else p.conteo = 0;
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

void actualizarPuertas() {
  if (emergencia) return;                 // en emergencia quedan abiertas
  if (millis() - tUltimaMedicion < PERIODO_MEDICION_MS) return;
  tUltimaMedicion = millis();
  puertaTurno ^= 1;                       // alterna para no mezclar ecos
  actualizarPuerta(puertas[puertaTurno]);
}

// =====================================================================
//                       EMERGENCIA Y RESET
// =====================================================================
void activarEmergencia(const char* origen) {
  if (emergencia) return;
  emergencia = true;
  tEmergencia = millis();
  for (auto& p : puertas) abrirPuerta(p);
  SONAR(PAT_EMERGENCIA);
  logEvento("EMERGENCIA activada (%s)", origen);
}

void resetSistema(const char* origen) {
  emergencia = false;
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
  if (leerBoton(botonReset) == 1) resetSistema("boton reset");
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
  j += ",\"oled\":false";
  j += ",\"max\":"; j += MAX_TRABAJADORES;
  j += ",\"tmax\":"; j += TIEMPO_MAX_PERMANENCIA_MS / 1000;
  j += ",\"humanos\":"; j += h;
  j += ",\"robots\":"; j += r;

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
  String ruta = "/f" + server.arg("v") + ".jpg";
  if (!LittleFS.exists(ruta)) { server.send(404, "text/plain", "sin foto"); return; }
  File f = LittleFS.open(ruta, "r");
  server.sendHeader("Cache-Control", "max-age=86400");
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleFotoPrueba() {
  if (!capturarJPEG("/prueba.jpg")) { server.send(500, "text/plain", "La camara no respondio"); return; }
  File f = LittleFS.open("/prueba.jpg", "r");
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

void iniciarWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/api/estado", HTTP_GET, handleEstado);
  server.on("/foto", HTTP_GET, handleFoto);
  server.on("/api/foto-prueba", HTTP_GET, handleFotoPrueba);
  server.on("/api/abrir", HTTP_POST, handleAbrir);
  server.on("/api/emergencia", HTTP_POST, handleEmergencia);
  server.on("/api/reset", HTTP_POST, handleReset);
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
  else if (c == "d") {
    for (auto& p : puertas)
      Serial.printf("%s: piso %.1f cm, altura objeto %.1f cm\n", p.nombre, p.alturaSensor, medirAltura(p));
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
  Serial.println(F("\n=== Control de acceso a planta - V2 (sin pantalla, dashboard) ==="));
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

  // Memoria para fotos
  if (!LittleFS.begin(true)) Serial.println(F("Error montando LittleFS"));
  borrarFotosViejas();

  // Cámara (antes del WiFi para tener RAM contigua)
  camaraOK = iniciarCamara();
  logEvento(camaraOK ? "Camara OV7670 lista" : "Camara NO detectada");

  // Calibración de los ultrasonidos (zona vacía)
  for (auto& p : puertas) calibrarSensor(p);

  iniciarWiFi();
  iniciarWeb();

  logEvento("Sistema listo");
  imprimirAyuda();
  SONAR(PAT_HUMANO);
}

void loop() {
  server.handleClient();
  leerBotones();
  actualizarPuertas();
  revisarTiempos();
  actualizarBuzzer();
  leerSerial();
}
