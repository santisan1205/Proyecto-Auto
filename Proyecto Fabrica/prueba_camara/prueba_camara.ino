/*
  =====================================================================
   PRUEBA DE LA CÁMARA OV7670 (sin FIFO) — solo la cámara, nada más
  =====================================================================
  Placa : ESP32 DevKit V1 (ESP32-WROOM-32, sin PSRAM)
  Core  : "esp32 by Espressif Systems" versión 3.x (probado con 3.3.8)

  Usa los MISMOS pines y la misma configuración que v1_con_oled.ino.
  No usa WiFi, OLED, sensores ni servos.

  Monitor Serie a 115200 baudios. Comandos (escribir y pulsar Enter):
    c  = tomar una foto y dibujarla en el monitor con caracteres
    a  = encender/apagar la captura automática (una foto cada 2 s)
    s  = revisar durante 1 s si cada señal de la cámara cambia
    i  = invertir el orden de bytes (si la imagen sale con ruido)
    p  = probar sola varias ventanas (líneas extra) hasta que la foto salga
    v N = pedir N líneas extra al final del cuadro (0 a 4), ej. "v 2"
    x N = cambiar el reloj XLK a N MHz y reiniciar la cámara, ej. "x 10"
    ?  = ver los comandos

  Conexiones (cámara -> ESP32):
    3V3 -> 3V3 (¡nunca 5 V!)   GND -> GND   RET -> 3V3   PWDN -> GND
    SCL -> 27 (+4.7k a 3V3)    SDA -> 26 (+4.7k a 3V3)
    VS  -> 34   HS -> 35   PLK -> 33   XLK -> 32
    D0  -> 19   D1 -> 18   D2  -> 17   D3  -> 5
    D4  -> 4    D5 -> 16   D6  -> 13   D7  -> 14
*/

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Este código requiere el core 'esp32 by Espressif Systems' versión 3.x"
#endif

#include "esp_camera.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_rom_uart.h"

// ---------------- Pines (iguales a v1_con_oled.ino) ----------------
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

// Reloj que el ESP32 le da a la cámara (pin XLK). Se cambia con "x N".
uint32_t camXclkHz = 6000000;
// Líneas extra que se le piden a la cámara al final de cada cuadro. Se
// cambia con "v N" o se busca sola con "p".
int lineasExtra = 1;   // 1 = probado: con 0 falla
// Captura automática
const uint32_t PERIODO_AUTO_MS = 2000;

bool     camaraOK       = false;
bool     invertirBytes  = false;
bool     auto_          = false;
uint32_t tAuto          = 0;
uint32_t fotosOK = 0, fotosFalla = 0;

// =====================================================================
//              FILTRO DE LOS MENSAJES "cam_hal: ..." DEL DRIVER
// =====================================================================
// El driver de la cámara imprime "cam_hal: FB-SIZE: ..." con la printf de
// la ROM cada vez que descarta un cuadro incompleto. Esta función se
// instala como salida de esa printf: deja pasar todo lo demás, no muestra
// las líneas que empiezan con "cam_hal:" y solo las cuenta.
volatile uint32_t cuadrosDescartados = 0;
static const char DRAM_ATTR PREFIJO_CAM[] = "cam_hal:";
static char    romBuf[sizeof(PREFIJO_CAM)];
static uint8_t romN = 0, romEstado = 0;   // 0 = decidiendo, 1 = mostrar, 2 = ocultar

void IRAM_ATTR filtroRomPutc(char ch) {
  if (romEstado == 0) {
    romBuf[romN++] = ch;
    if (ch == PREFIJO_CAM[romN - 1]) {
      if (romN < sizeof(PREFIJO_CAM) - 1) return;    // por ahora coincide
      romEstado = 2; romN = 0; return;                // es un mensaje del driver
    }
    for (uint8_t i = 0; i < romN; i++) esp_rom_output_putc(romBuf[i]);   // no lo era
    romN = 0;
    romEstado = (ch == '\n') ? 0 : 1;
    return;
  }
  if (romEstado == 1) esp_rom_output_putc(ch);
  if (ch == '\n') {
    if (romEstado == 2) cuadrosDescartados++;
    romEstado = 0;
  }
}

// =====================================================================
//                              CÁMARA
// =====================================================================
// Funciones del driver de la cámara para leer/escribir sus registros
extern "C" uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
extern "C" int     SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);

// El driver deja la ventana vertical en 478 filas: salen 119 líneas en vez
// de 120 y todos los cuadros se descartan ("FB-SIZE: 30720 != 38400").
// Se deja en 480 filas (10..490) para que salgan las 120 líneas, más
// 'lineasExtra' líneas (4 filas del sensor por línea) al final del cuadro.
void corregirVentanaOV7670() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s || s->id.PID != OV7670_PID) return;
  const uint16_t vInicio = 10, vFin = 490 + 4 * lineasExtra;
  SCCB_Write(s->slv_addr, 0x19, vInicio >> 2);                          // VSTRT
  SCCB_Write(s->slv_addr, 0x1A, vFin >> 2);                             // VSTOP
  SCCB_Write(s->slv_addr, 0x03, ((vFin & 3) << 2) | (vInicio & 3));     // VREF
  Serial.printf("Ventana vertical: filas %u-%u, %d lineas extra (VSTRT=0x%02X VSTOP=0x%02X VREF=0x%02X)\n",
                vInicio, vFin, lineasExtra,
                SCCB_Read(s->slv_addr, 0x19), SCCB_Read(s->slv_addr, 0x1A), SCCB_Read(s->slv_addr, 0x03));
}

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
  c.xclk_freq_hz = camXclkHz;
  c.ledc_timer   = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_RGB565;    // la OV7670 no genera JPEG
  c.frame_size   = FRAMESIZE_QQVGA;     // 160x120
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.sccb_i2c_port = 1;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camara: error 0x%x al iniciar.\n", err);
    Serial.println(F("=> No responde por SCL/SDA. Revisa 3V3, GND, PWDN a GND, RET a 3V3, XLK (32) y las pull-ups de 4.7k en 26 y 27."));
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  Serial.printf("Camara detectada: PID 0x%02X (OV7670 = 0x76)\n", s ? s->id.PID : 0);
  corregirVentanaOV7670();
  return true;
}

// Brillo (0-255) de un píxel RGB565
uint8_t brillo(const uint8_t* p) {
  uint16_t v = invertirBytes ? (p[1] << 8) | p[0] : (p[0] << 8) | p[1];
  uint8_t r = (v >> 11) << 3, g = ((v >> 5) & 0x3F) << 2, b = (v & 0x1F) << 3;
  return (r * 77 + g * 150 + b * 29) >> 8;
}

// Dibuja la imagen en el Monitor Serie: 80x30 caracteres (bloques de 2x4 píxeles)
void dibujarAscii(camera_fb_t* fb) {
  static const char rampa[] = " .:-=+*#%@";
  const int W = fb->width, H = fb->height, bx = 2, by = 4;
  Serial.print('+'); for (int x = 0; x < W / bx; x++) Serial.print('-'); Serial.println('+');
  for (int y = 0; y + by <= H; y += by) {
    Serial.print('|');
    for (int x = 0; x + bx <= W; x += bx) {
      uint16_t suma = 0;
      for (int j = 0; j < by; j++)
        for (int i = 0; i < bx; i++)
          suma += brillo(fb->buf + ((y + j) * W + (x + i)) * 2);
      uint8_t m = suma / (bx * by);
      Serial.print(rampa[m * (sizeof(rampa) - 2) / 255]);
    }
    Serial.println('|');
  }
  Serial.print('+'); for (int x = 0; x < W / bx; x++) Serial.print('-'); Serial.println('+');
}

void tomarFoto(bool dibujar) {
  if (!camaraOK) { Serial.println(F("La camara no inicio. Revisa las conexiones y reinicia (boton EN).")); return; }
  uint32_t t0 = millis();
  camera_fb_t* fb = esp_camera_fb_get();          // descarta el cuadro viejo
  if (fb) {                                       // (si falla, no espera otros 4 s)
    esp_camera_fb_return(fb);
    t0 = millis();
    fb = esp_camera_fb_get();
  }
  uint32_t dt = millis() - t0;
  if (!fb) {
    fotosFalla++;
    Serial.printf("FOTO: FALLA, no llego ningun cuadro completo (%lu ms). OK %lu / FALLA %lu\n",
                  (unsigned long)dt, (unsigned long)fotosOK, (unsigned long)fotosFalla);
    Serial.printf("=> El driver lleva %lu cuadros incompletos descartados. Prueba 'p' o 's'.\n",
                  (unsigned long)cuadrosDescartados);
    return;
  }
  fotosOK++;
  uint32_t suma = 0; uint8_t mn = 255, mx = 0;
  for (size_t i = 0; i + 1 < fb->len; i += 2) {
    uint8_t b = brillo(fb->buf + i);
    suma += b; if (b < mn) mn = b; if (b > mx) mx = b;
  }
  uint32_t n = fb->len / 2;
  Serial.printf("FOTO: OK %ux%u, %u bytes, en %lu ms | brillo medio %lu (min %u, max %u) | OK %lu / FALLA %lu\n",
                fb->width, fb->height, (unsigned)fb->len, (unsigned long)dt,
                (unsigned long)(n ? suma / n : 0), mn, mx, (unsigned long)fotosOK, (unsigned long)fotosFalla);
  if (mx - mn < 10)
    Serial.println(F("=> La imagen es casi plana (todo igual). Destapa el lente, dale luz o revisa D0-D7."));
  if (dibujar) dibujarAscii(fb);
  esp_camera_fb_return(fb);
}

// Prueba de 0 a 4 líneas extra y dice con cuál llega la foto completa.
// Cada intento fallido tarda unos 4 s (el driver espera ese tiempo).
void probarVentanas() {
  if (!camaraOK) { Serial.println(F("La camara no inicio.")); return; }
  Serial.printf("Probando ventanas con XLK a %lu MHz (hasta ~20 s)...\n", (unsigned long)(camXclkHz / 1000000));
  int primera = -1;
  for (int n = 0; n <= 4; n++) {
    lineasExtra = n;
    corregirVentanaOV7670();
    delay(200);
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    Serial.printf("  %d lineas extra: %s\n", n, fb ? "FOTO OK" : "falla");
    if (fb) { primera = n; break; }
  }
  if (primera >= 0) {
    Serial.printf("=> Funciona con %d lineas extra.\n", primera);
    tomarFoto(true);
  } else {
    lineasExtra = 0;
    corregirVentanaOV7670();
    Serial.println(F("=> Ninguna ventana sirvio. Prueba otro reloj: \"x 10\" y luego \"p\" otra vez."));
  }
}

// Cambia el reloj XLK y reinicia la cámara
void cambiarReloj(int mhz) {
  if (mhz < 4 || mhz > 24) { Serial.println(F("Usa un valor entre 4 y 24 MHz.")); return; }
  if (camaraOK) esp_camera_deinit();
  camXclkHz = (uint32_t)mhz * 1000000UL;
  Serial.printf("Reiniciando la camara con XLK a %d MHz...\n", mhz);
  camaraOK = iniciarCamara();
  if (camaraOK) { delay(300); tomarFoto(false); }
}

// Durante 1 s mira si cada señal de la cámara cambia
void revisarSenales() {
  if (!camaraOK) { Serial.println(F("La camara no inicio: primero revisa SCL/SDA, 3V3, GND y XLK.")); return; }
  const uint8_t pines[] = { CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK,
                            CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2, CAM_PIN_D3,
                            CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7 };
  const char* nombres[] = { "VS ", "HS ", "PLK", "D0 ", "D1 ", "D2 ", "D3 ",
                            "D4 ", "D5 ", "D6 ", "D7 " };
  const int N = sizeof(pines);
  bool vio0[N] = {}, vio1[N] = {};
  uint32_t cuadros = 0, lineas = 0, k = 0;
  int vsAnt = gpio_get_level((gpio_num_t)CAM_PIN_VSYNC);
  int hsAnt = gpio_get_level((gpio_num_t)CAM_PIN_HREF);
  uint32_t t0 = millis();
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
  Serial.printf("Senales durante 1 s (XLK a %lu MHz):\n", (unsigned long)(camXclkHz / 1000000));
  for (int i = 0; i < N; i++) {
    const char* estado = (vio0[i] && vio1[i]) ? "cambia (bien)" : vio1[i] ? "FIJA en 1" : "FIJA en 0";
    Serial.printf("  %s GPIO %2d: %s\n", nombres[i], pines[i], estado);
  }
  Serial.printf("  Cuadros por segundo (VS): %lu   Lineas por cuadro (HS): ~%lu (deben ser 120)\n",
                (unsigned long)cuadros, (unsigned long)(cuadros ? lineas / cuadros : 0));
  bool vsOK = vio0[0] && vio1[0], hsOK = vio0[1] && vio1[1], plkOK = vio0[2] && vio1[2];
  if (!vsOK && !hsOK && !plkOK)
    Serial.println(F("=> No sale nada de la camara: revisa XLK (GPIO 32), su alimentacion y PWDN a GND."));
  else if (!plkOK)
    Serial.println(F("=> Falla PLK (GPIO 33): cable suelto, mal puesto o muy largo."));
  else if (!vsOK || !hsOK)
    Serial.println(F("=> Falla VS (34) o HS (35): revisa ese cable y que no esten intercambiados."));
  else {
    bool datosFijos = false;
    for (int i = 3; i < N; i++) if (!(vio0[i] && vio1[i])) datosFijos = true;
    Serial.println(datosFijos
      ? F("=> Algun D0-D7 no cambia. Apunta la camara a algo con luz y repite; si sigue fijo, revisa ese cable.")
      : F("=> Todas las senales llegan."));
  }
}

// =====================================================================
//                           MONITOR SERIE
// =====================================================================
void ayuda() {
  Serial.println(F("Comandos: c = tomar foto y dibujarla | a = foto automatica cada 2 s (on/off)"));
  Serial.println(F("          s = revisar senales | i = invertir bytes | p = probar ventanas solo"));
  Serial.println(F("          v N = N lineas extra (0-4) | x N = reloj a N MHz | ? = ayuda"));
}

void leerSerial() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim(); c.toLowerCase();
  if (c.length() == 0) return;
  if      (c == "c") tomarFoto(true);
  else if (c == "a") { auto_ = !auto_; tAuto = 0; Serial.printf("Foto automatica: %s\n", auto_ ? "ENCENDIDA" : "apagada"); }
  else if (c == "s") revisarSenales();
  else if (c == "p") probarVentanas();
  else if (c.startsWith("v ")) {
    int n = c.substring(2).toInt();
    if (n < 0 || n > 4) { Serial.println(F("Usa un valor entre 0 y 4.")); return; }
    lineasExtra = n;
    corregirVentanaOV7670();
    delay(200);
    tomarFoto(false);
  }
  else if (c.startsWith("x ")) cambiarReloj(c.substring(2).toInt());
  else if (c == "i") { invertirBytes = !invertirBytes; Serial.printf("Invertir bytes: %s\n", invertirBytes ? "SI" : "no"); }
  else ayuda();
}

// =====================================================================
//                              SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  esp_rom_install_channel_putc(1, filtroRomPutc);   // oculta los "cam_hal: ..."
  delay(500);
  Serial.println();
  Serial.println(F("===== PRUEBA DE CAMARA OV7670 ====="));
  camaraOK = iniciarCamara();
  if (camaraOK) {
    delay(300);
    tomarFoto(true);
  }
  ayuda();
}

void loop() {
  leerSerial();
  if (auto_ && millis() - tAuto >= PERIODO_AUTO_MS) {
    tAuto = millis();
    tomarFoto(true);
  }
}
