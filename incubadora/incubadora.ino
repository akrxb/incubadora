#include <LovyanGFX.hpp>
#include <Wire.h>
#include "SHTSensor.h"
#include <Adafruit_HX711.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
// ============================================================
// --- Buses I2C ---
// ============================================================
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

// ── WiFi ──────────────────────────────────────────────────────
static const char* AP_SSID      = "BioSense-AP";
static const char* AP_PASSWORD  = "biosense1234";
static const char* STA_SSID     = "ssid";
static const char* STA_PASSWORD = "contraseña";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

static const uint32_t WS_PERIOD_MS = 1000;
uint32_t lastWsMs = 0;


// Pines I2C — adaptados al ESP32-S3 del proyecto principal
#define SDA1 7
#define SCL1 15
#define SDA2 16
#define SCL2 17

// ============================================================
// --- Sensores ---
// ============================================================
 SHTSensor shtc3_1; 
 SHTSensor shtc3_2;
SCD4x scd41;

// ============================================================
// --- Configuración de Hardware (Display + Touch) ---
// ============================================================
class LGFX_ESP32S3_Custom : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796    _panel_instance;
  lgfx::Bus_SPI         _bus_instance;
  lgfx::Touch_XPT2046   _touch_instance;

  public:
  LGFX_ESP32S3_Custom() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 40000000;
      cfg.pin_sclk   = 12;
      cfg.pin_mosi   = 11;
      cfg.pin_miso   = 13;
      cfg.pin_dc     = 9;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs       = 10;
      cfg.pin_rst      = 8;
      cfg.panel_width  = 320;
      cfg.panel_height = 480;
      cfg.bus_shared   = true;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _touch_instance.config();
      cfg.pin_cs     = 14;
      cfg.bus_shared = true;
      cfg.spi_host   = SPI2_HOST;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX_ESP32S3_Custom tft;

// ============================================================
// --- Paleta de Colores ---
// ============================================================
static const uint32_t C_BG        = 0x000000;
static const uint32_t C_WHITE     = 0xFFFFFF;
static const uint32_t C_PRIMARY   = 0x0056B3;
static const uint32_t C_ACCENT    = 0x00D1B2;
static const uint32_t C_LTBLUE    = 0xF1F4F8;
static const uint32_t C_TEXT_DARK = 0x2C3E50;
static const uint32_t C_RED       = 0xFF0000;
static const uint32_t C_OFF       = 0xD3D3D3;

// ============================================================
// --- Variables de Estado ---
// ============================================================
enum Estado { CALIBRACION, SPLASH, MENU, PESO, LEDS, TEMP, AMBIENTE };
Estado estadoActual = CALIBRACION;

// --- AMBIENTE ---
float humedadActual         = 0;
float co2Actual             = 0;
unsigned long ultimoUpdateAmbiente = 0;
unsigned long ultimoUpdateCO2     = 0;

// --- TEMPERATURA + PID ---
float tempActual   = 0.0;
float tempConsigna = 30.0;          // Setpoint (°C) — editable desde pantalla
unsigned long ultimoUpdateTemp = 0;

// Datos crudos SHTC3
float temp1Real = NAN, hum1Real = NAN;
float temp2Real = NAN, hum2Real = NAN;

// PID
#define PWM_PIN_CALEFACTOR 4        // Pin PWM del elemento calefactor
float Kp = 20.0;                    // Ajusta según tu sistema
float Ki = 0.05;
float Kd = 2.0;
float pid_I            = 0.0;
float pid_P            = 0.0;
float pid_D            = 0.0;
float temp_anterior    = 0.0;
float salida_porcentaje = 0.0;
unsigned long tiempo_anterior = 0;

// --- PESO ---
#define DT_PESO1  3
#define SCK_PESO1  18
#define DT_PESO2  41
#define SCK_PESO2  40
Adafruit_HX711 HX711_PESO1(DT_PESO1, SCK_PESO1);
Adafruit_HX711 HX711_PESO2(DT_PESO2,  SCK_PESO2);
// Sensor 1: dos tramos
#define M1_LOW  0.0027
#define B1_LOW  -1330.2
#define M1_HIGH 0.0058
#define B1_HIGH -6166.6
#define UMBRAL1 1500043

// Sensor 2: una sola recta  <-- cambia estos valores
#define M2  0.0025
#define B2  111.55
float alpha = 0.3;
float peso_filtrado1 = 0;
float peso_filtrado2 = 0;
float tara1 = 0;
float tara2 = 0;
float peso_total = 0;
unsigned long ultimoUpdatePeso = 0;

// --- LEDS ---
bool     ledEncendido   = false;
uint32_t colorActualLed = C_PRIMARY;
int      brilloLed      = 50;
#define PIN_LED_AZUL 5
#define PIN_LED_ROJO 6


uint32_t lastTouchMs = 0;
#define TOUCH_DEBOUNCE 250   // ms mínimo entre pulsaciones
// ============================================================
// --- Prototipos ---
// ============================================================
void  irAMenu();
void  irASplash();
void  aplicarEstadoLed();
float calcularPeso1(int32_t raw);
float calcularPeso2(int32_t raw);
void  pantallaPeso();
void  pantallaLeds();
void  pantallaTemp();
void  pantallaAmbiente();
void  actualizarPantallaAmbiente();
void  leerSensoresAmbiente();
void  dibujarHospitalGrande(int x, int y);
void  dibujarBotonCuadrado(int x, int y, const char* txt);
void  dibujarCabecera(const char* titulo, bool mostrarVolver);
void  dibujarIconoBalanza(int x, int y);
void  dibujarIconoBombilla(int x, int y, uint32_t color, bool encendido);
void  dibujarIconoGota(int x, int y, uint32_t color);
void  dibujarIconoCO2(int x, int y, uint32_t color);
void  actualizarPantallaLeds();
void  dibujarSlider(int x, int y, int ancho, int valor);
void  actualizarLecturaPeso();
void  actualizarPantallaTemp();
void  update_PID();
float leer_temperatura();

// ============================================================
// --- Setup ---
// ============================================================
void setup() {
  Serial.begin(115200);
  tft.init();

  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);

  // PWM para el calefactor (canal LEDC, 500 Hz, 8 bits)
  ledcAttach(PWM_PIN_CALEFACTOR, 500, 8);

  HX711_PESO1.begin();
  HX711_PESO2.begin();
  // Buses I2C
  I2C_1.begin(SDA1, SCL1);
  I2C_2.begin(SDA2, SCL2);

  // SHTC3
 if (!shtc3_1.init(I2C_1)) Serial.println("Error SHTC3 #1"); 
if (!shtc3_2.init(I2C_2)) Serial.println("Error SHTC3 #2");

  // SCD41
  if (!scd41.begin(I2C_1)) {
    Serial.println("Error SCD41");
  } else {
    scd41.startPeriodicMeasurement();
  }

  // Inicializar PID
  tiempo_anterior = millis();
  temp_anterior   = 0.0;

  // Pantalla
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_WHITE);
  tft.drawCenterString("CALIBRACION", 240, 140);
  tft.calibrateTouch(nullptr, C_ACCENT, C_BG, 20);


  // ── LittleFS ──────────────────────────────────────────────────
  if (!LittleFS.begin(true))
    Serial.println("[FS] Error LittleFS");
  else
    Serial.println("[FS] LittleFS OK");

  // ── WiFi dual AP + STA ────────────────────────────────────────
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(STA_SSID, STA_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[WiFi] STA -> %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[WiFi] Solo AP");

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[WiFi] AP -> %s\n", WiFi.softAPIP().toString().c_str());

  // ── Servidor web + WebSocket ───────────────────────────────────
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildJson());
  });
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("[HTTP] Servidor en :80");


  irASplash();
}
// ── Construye el JSON con datos reales ────────────────────────
String buildJson() {
  JsonDocument doc;
  doc["temp1"]    = isnan(temp1Real) ? 0.0f : round(temp1Real * 10) / 10.0f;
  doc["temp2"]    = isnan(temp2Real) ? 0.0f : round(temp2Real * 10) / 10.0f;
  doc["hum1"]     = isnan(hum1Real)  ? 0.0f : round(hum1Real  * 10) / 10.0f;
  doc["hum2"]     = isnan(hum2Real)  ? 0.0f : round(hum2Real  * 10) / 10.0f;
  doc["co2"]      = (int)co2Actual;
  doc["setpoint"] = round(tempConsigna * 10) / 10.0f;
  doc["weight"]   = round((peso_total / 1000.0f) * 1000) / 1000.0f;
  doc["ledRed"]   = ledEncendido && (colorActualLed == C_RED);
  doc["ledBlue"]  = ledEncendido && (colorActualLed == C_PRIMARY);
  doc["ledInt"]   = ledEncendido ? brilloLed : 0;
  doc["uptime"]   = millis() / 1000;
  String out;
  serializeJson(doc, out);
  return out;
}

// ── Evento WebSocket ──────────────────────────────────────────
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text(buildJson());  // envía estado actual al conectar
  }
}
// ============================================================
// --- Loop Principal ---
// ============================================================
void loop() {
  uint16_t x, y;
  bool touched = tft.getTouch(&x, &y);

  // Lectura de sensores + PID siempre activos
  leerSensoresAmbiente();
  update_PID();
  // ── Envío periódico de datos reales por WebSocket ─────────────
    if (millis() - lastWsMs >= WS_PERIOD_MS) {
      lastWsMs = millis();
      ws.cleanupClients();
      ws.textAll(buildJson());
    }
  switch (estadoActual) {

    case SPLASH:
      if (touched) {
        delay(300);
        irAMenu();
      }
      break;

    case MENU:
      if (touched) {
        if (x > 30 && x < 230) {
          if (y > 80  && y < 180) { pantallaPeso();    }
          if (y > 200 && y < 300) { pantallaLeds();    }
        }
        if (x > 250 && x < 450) {
          if (y > 80  && y < 180) { pantallaTemp();    }
          if (y > 200 && y < 300) { pantallaAmbiente();}
        }
        delay(250);
      }
      break;

    case PESO:
      actualizarLecturaPeso();
      if (touched && x < 120 && y < 50) {
        delay(250);
        irAMenu();
      }
      // Botón TARA
      if (touched && x > 300 && x < 420 && y > 240 && y < 290) {

        tara1 = peso_filtrado1;
        tara2 = peso_filtrado2;

        Serial.println(">>> Tara aplicada <<<");

        delay(300);
      }
      break;

    case LEDS:
      if (touched) {
        if (x < 120 && y < 50) {
          delay(250);
          irAMenu();
        }
        if (x > 40 && x < 140) {
          if (y > 70 && y < 120) {
            ledEncendido = true;
            actualizarPantallaLeds();
            aplicarEstadoLed();
            delay(150);
          } else if (y > 130 && y < 180) {
            ledEncendido = false;
            actualizarPantallaLeds();
            aplicarEstadoLed();
            delay(150);
          }
        }
        if (y > 190 && y < 230) {
          if (x > 50 && x < 80) {
            colorActualLed = C_RED;
            actualizarPantallaLeds();
            aplicarEstadoLed();
            delay(150);
          } else if (x > 100 && x < 130) {
            colorActualLed = C_PRIMARY;
            actualizarPantallaLeds();
            aplicarEstadoLed();
            delay(150);
          }
        }
        if (x >= 180 && x <= 420 && y > 240 && y < 300) {
          int nuevoBrillo = map(x, 180, 420, 0, 100);
          if (nuevoBrillo != brilloLed) {
            int posAnterior = map(brilloLed, 0, 100, 0, 240);
            tft.fillCircle(180 + posAnterior, 275, 15, C_LTBLUE);
            brilloLed = nuevoBrillo;
            dibujarSlider(180, 270, 240, brilloLed);
            aplicarEstadoLed();
          }
        }
      }
      break;

    case TEMP:
      // La pantalla se actualiza automáticamente en update_PID()
      if (touched) {
        if (x < 120 && y < 50) {
          delay(200);
          irAMenu();
        }
        if (y > 200 && y < 260) {
          if (x > 220 && x < 310) {   // Botón MÁS
            tempConsigna += 0.5;
            actualizarPantallaTemp();
            delay(150);
          }
          if (x > 325 && x < 415) {   // Botón MENOS
            tempConsigna -= 0.5;
            actualizarPantallaTemp();
            delay(150);
          }
        }
      }
      break;

    case AMBIENTE:
      if (touched && x < 120 && y < 50) {
        delay(250);
        irAMenu();
      }
      break;
  }
}

// ============================================================
// --- PID ---
// ============================================================

float leerTemperaturaSensor() {
  return tempActual; // Ya actualizada por leerSensoresAmbiente()
}

// Algoritmo PID — se llama en cada iteración del loop
void update_PID() {
  unsigned long ahora = millis();

  // Ejecutar solo cada 500 ms para no saturar los sensores
  if (ahora - ultimoUpdateTemp < 500) return;
  float dt = (ahora - tiempo_anterior) / 1000.0;
  tiempo_anterior = ahora;
  ultimoUpdateTemp = ahora;

  tempActual = leerTemperaturaSensor();
  // Guardamos humedad media de paso (reutiliza la última lectura de ambiente)

  if (dt > 0) {
    float error = tempConsigna - tempActual;   // error positivo → hay que calentar

    pid_P = Kp * error;

    pid_I += Ki * error * dt;
    pid_I  = constrain(pid_I, 0.0, 255.0);    // Anti-windup

    pid_D = -Kd * (tempActual - temp_anterior) / dt;

    float salida = constrain(pid_P + pid_I + pid_D, 0.0, 255.0);
    salida_porcentaje = (salida / 255.0) * 100.0;

    temp_anterior = tempActual;

    ledcWrite(PWM_PIN_CALEFACTOR, (int)salida);

    // Debug por puerto serie
    Serial.printf("T=%.2f  SP=%.2f  P=%.2f  I=%.2f  D=%.2f  OUT=%.1f%%\n",
                  tempActual, tempConsigna, pid_P, pid_I, pid_D, salida_porcentaje);
  }

  // Refrescar pantalla si está visible
  if (estadoActual == TEMP) {
    actualizarPantallaTemp();
  }
}

// ============================================================
// --- Lectura de Sensores de Ambiente ---
// ============================================================

void leerSensoresAmbiente() {
  unsigned long ahora = millis();

  // SHTC3 cada 2 s
  if (ahora - ultimoUpdateAmbiente >= 2000) {
    ultimoUpdateAmbiente = ahora;

    // Sensor 1
    if (shtc3_1.readSample()) {
      temp1Real = shtc3_1.getTemperature();
      hum1Real  = shtc3_1.getHumidity();
    }
    // Sensor 2
    if (shtc3_2.readSample()) {
      temp2Real = shtc3_2.getTemperature();
      hum2Real  = shtc3_2.getHumidity();
    }

    // Humedad: media de los dos sensores
    if      (!isnan(hum1Real) && !isnan(hum2Real)) humedadActual = (hum1Real + hum2Real) / 2.0;
    else if (!isnan(hum1Real))                      humedadActual = hum1Real;
    else if (!isnan(hum2Real))                      humedadActual = hum2Real;

    // Temperatura: media de los dos
    if      (!isnan(temp1Real) && !isnan(temp2Real)) tempActual = (temp1Real + temp2Real) / 2.0;
    else if (!isnan(temp1Real))                       tempActual = temp1Real;
    else if (!isnan(temp2Real))                       tempActual = temp2Real;

    if (estadoActual == AMBIENTE) actualizarPantallaAmbiente();
  }

  // SCD41 cada 5 s
  if (ahora - ultimoUpdateCO2 >= 5000) {
    ultimoUpdateCO2 = ahora;
    if (scd41.readMeasurement()) {
      co2Actual = scd41.getCO2();
      if (estadoActual == AMBIENTE) actualizarPantallaAmbiente();
    }
  }
}

// ============================================================
// --- Pantallas ---
// ============================================================

void irASplash() {
  estadoActual = SPLASH;
  tft.fillScreen(C_BG);
  dibujarHospitalGrande(240, 145);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(3);
  tft.drawCenterString("AREA DE",     240, 205);
  tft.drawCenterString("INCUBADORAS", 240, 240);
  tft.setTextSize(1);
  tft.setTextColor(C_ACCENT);
  tft.drawCenterString("PULSE PARA ACCEDER", 240, 295);
}

void aplicarEstadoLed() {
  int pwmValor = ledEncendido ? map(brilloLed, 0, 100, 0, 255) : 0;
  if (!ledEncendido) {
    analogWrite(PIN_LED_AZUL, 0);
    analogWrite(PIN_LED_ROJO, 0);
  } else {
    if (colorActualLed == C_PRIMARY) {
      analogWrite(PIN_LED_AZUL, pwmValor);
      analogWrite(PIN_LED_ROJO, 0);
    } else if (colorActualLed == C_RED) {
      analogWrite(PIN_LED_ROJO, pwmValor);
      analogWrite(PIN_LED_AZUL, 0);
    }
  }
}

float calcularPeso1(int32_t raw) {
  if (raw >= UMBRAL1) {
    return (raw * M1_HIGH) + B1_HIGH;
  } else {
    return (raw * M1_LOW) + B1_LOW;
  }
}

float calcularPeso2(int32_t raw) {
  return (raw * M2) + B2;
}

void irAMenu() {
  estadoActual = MENU;
  tft.fillScreen(C_LTBLUE);
  dibujarCabecera("MENU PRINCIPAL", false);
  dibujarBotonCuadrado(30,  80,  "PESO");
  dibujarBotonCuadrado(30,  200, "LEDS");
  dibujarBotonCuadrado(250, 80,  "TEMPERATURA");
  dibujarBotonCuadrado(250, 200, "CALIDAD AIRE");
}

void pantallaPeso() {
  estadoActual = PESO;
  tft.fillScreen(C_LTBLUE);
  dibujarCabecera("MONITOR DE PESO", true);

  dibujarIconoBalanza(130, 180);

  tft.fillRoundRect(270, 100, 180, 120, 10, C_BG);
  tft.drawRoundRect(270, 100, 180, 120, 10, C_PRIMARY);

  tft.setTextColor(C_ACCENT);
  tft.setTextSize(2);
  tft.drawCenterString("PESO", 360, 115);

  // Botón TARA
  tft.fillRoundRect(300, 240, 120, 50, 10, C_PRIMARY);
  tft.drawRoundRect(300, 240, 120, 50, 10, C_WHITE);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.drawCenterString("TARA", 360, 257);

  ultimoUpdatePeso = 0;
}

void pantallaLeds() {
  estadoActual = LEDS;
  tft.fillScreen(C_LTBLUE);
  dibujarCabecera("CONTROL LEDS", true);
  actualizarPantallaLeds();
}

void pantallaTemp() {
  estadoActual = TEMP;
  tft.fillScreen(C_LTBLUE);
  dibujarCabecera("CONTROL TEMP", true);

  int tX   = 80;
  int tY   = 260;
  int btnY = 200;

  // Estructura del termómetro (estático)
  tft.fillCircle(tX, tY, 35, C_TEXT_DARK);
  tft.fillRoundRect(tX - 12, 80, 24, 180, 12, C_TEXT_DARK);
  tft.fillCircle(tX, tY, 30, C_WHITE);
  tft.fillRoundRect(tX - 8, 85, 16, 175, 8, C_WHITE);

  // Marcas de graduación
  tft.setTextSize(1);
  tft.setTextColor(C_TEXT_DARK);
  for (int i = 10; i <= 40; i += 10) {
    int marcaY = map(i, 0, 40, 250, 90);
    tft.drawFastHLine(tX + 14, marcaY, 8, C_TEXT_DARK);
    tft.drawNumber(i, tX + 25, marcaY - 4);
  }

  // Botones + y -
  tft.fillRoundRect(220, btnY, 90, 60, 12, C_RED);
  tft.fillRoundRect(325, btnY, 90, 60, 12, C_PRIMARY);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(3);
  tft.drawCenterString("+", 265, btnY + 20);
  tft.drawCenterString("-", 370, btnY + 20);

  actualizarPantallaTemp();
}

void pantallaAmbiente() {
  estadoActual = AMBIENTE;
  tft.fillScreen(C_LTBLUE);
  dibujarCabecera("CALIDAD AIRE", true);

  // Icono HUMEDAD
  tft.fillCircle(130, 115, 26, C_PRIMARY);
  tft.fillTriangle(130, 68, 110, 115, 150, 115, C_PRIMARY);
  tft.fillCircle(119, 105, 7, C_WHITE);
  tft.setTextColor(C_TEXT_DARK);
  tft.setTextSize(1);
  tft.drawCenterString("HUMEDAD", 130, 148);

  // Icono CO2
  tft.fillCircle(350, 108, 22, C_PRIMARY);
  tft.fillCircle(314, 108, 14, C_ACCENT);
  tft.fillCircle(386, 108, 14, C_ACCENT);
  tft.drawFastHLine(328, 108, 22, C_WHITE);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.drawCenterString("CO2", 350, 104);
  tft.setTextColor(C_TEXT_DARK);
  tft.drawCenterString("CO2 / AIRE", 350, 130);

  // Tarjeta HUMEDAD
  tft.fillRoundRect(40,  160, 180, 100, 15, C_BG);
  tft.drawRoundRect(40,  160, 180, 100, 15, C_PRIMARY);
  tft.setTextColor(C_ACCENT);
  tft.setTextSize(1);
  tft.drawCenterString("% HR", 130, 243);

  // Tarjeta CO2
  tft.fillRoundRect(260, 160, 180, 100, 15, C_BG);
  tft.drawRoundRect(260, 160, 180, 100, 15, C_PRIMARY);
  tft.setTextColor(C_ACCENT);
  tft.setTextSize(1);
  tft.drawCenterString("ppm", 350, 243);

  ultimoUpdateAmbiente = 0;
}

// ============================================================
// --- Actualización de Pantallas ---
// ============================================================

void actualizarPantallaAmbiente() {
  // HUMEDAD
  tft.fillRect(50, 168, 160, 65, C_BG);

  uint32_t colorHum = C_ACCENT;
  if (humedadActual < 45 || humedadActual > 75) colorHum = 0xFFA500;
  if (humedadActual < 30 || humedadActual > 85) colorHum = C_RED;

  tft.setTextColor(C_WHITE);
  tft.setTextSize(3);
  char bufHum[8];
  sprintf(bufHum, "%.1f", humedadActual);
  tft.drawCenterString(bufHum, 130, 185);

  tft.setTextSize(1);
  tft.fillRect(50, 223, 160, 12, C_BG);
  tft.setTextColor(colorHum);
  if      (colorHum == C_ACCENT) tft.drawCenterString("OPTIMA",  130, 224);
  else if (colorHum == 0xFFA500) tft.drawCenterString("REVISAR", 130, 224);
  else                           tft.drawCenterString("ALERTA",  130, 224);
  tft.setTextColor(C_ACCENT);
  tft.drawCenterString("% HR", 130, 243);

  // CO2
  tft.fillRect(270, 168, 160, 65, C_BG);

  uint32_t colorCO2 = C_ACCENT;
  if (co2Actual > 800)  colorCO2 = 0xFFA500;
  if (co2Actual > 1000) colorCO2 = C_RED;

  tft.setTextColor(C_WHITE);
  tft.setTextSize(3);
  char bufCO2[8];
  sprintf(bufCO2, "%d", (int)co2Actual);
  tft.drawCenterString(bufCO2, 350, 185);

  tft.setTextSize(1);
  tft.fillRect(270, 223, 160, 12, C_BG);
  tft.setTextColor(colorCO2);
  if      (colorCO2 == C_ACCENT) tft.drawCenterString("BUENA CALIDAD", 350, 224);
  else if (colorCO2 == 0xFFA500) tft.drawCenterString("CALIDAD MEDIA", 350, 224);
  else                           tft.drawCenterString("CALIDAD MALA",  350, 224);
  tft.setTextColor(C_ACCENT);
  tft.drawCenterString("ppm", 350, 243);
}

void actualizarPantallaLeds() {
  int cX = 220, cY = 90, cW = 160, cH = 120;

  tft.fillRoundRect(cX + 4, cY + 4, cW, cH, 15, 0xBDC3C7);
  tft.fillRoundRect(cX, cY, cW, cH, 15, C_WHITE);
  tft.drawRoundRect(cX, cY, cW, cH, 15, C_PRIMARY);
  dibujarIconoBombilla(cX + (cW / 2), cY + (cH / 2) + 10,
                       ledEncendido ? colorActualLed : C_OFF, ledEncendido);

  tft.setTextSize(2);
  tft.fillRoundRect(40, 70,  100, 50, 8, ledEncendido  ? C_ACCENT    : C_TEXT_DARK);
  tft.setTextColor(C_WHITE);
  tft.drawCenterString("ON",  90, 85);
  tft.fillRoundRect(40, 130, 100, 50, 8, !ledEncendido ? C_RED       : C_TEXT_DARK);
  tft.drawCenterString("OFF", 90, 145);

  tft.fillCircle(65,  210, 15, C_RED);
  if (colorActualLed == C_RED)     tft.drawCircle(65,  210, 19, C_TEXT_DARK);
  tft.fillCircle(115, 210, 15, C_PRIMARY);
  if (colorActualLed == C_PRIMARY) tft.drawCircle(115, 210, 19, C_TEXT_DARK);

  dibujarSlider(180, 270, 240, brilloLed);
}

void dibujarSlider(int x, int y, int ancho, int valor) {
  tft.fillRoundRect(x, y, ancho, 10, 5, 0xD6DBDF);
  tft.fillRoundRect(x, y, map(valor, 0, 100, 0, ancho), 10, 5, C_PRIMARY);
  int pos = map(valor, 0, 100, 0, ancho);
  tft.fillCircle(x + pos, y + 5, 14, C_WHITE);
  tft.drawCircle(x + pos, y + 5, 14, C_PRIMARY);
}

void actualizarPantallaTemp() {
  int tX         = 80;
  int tY         = 260;
  int controlX   = 210;
  int controlY   = 90;
  int separacionX = 120;

  // Mercurio del termómetro
  int pxAlto = map((int)(tempActual * 10), 0, 400, 0, 160);
  tft.fillRect(tX - 5, 85, 10, 165 - pxAlto, C_WHITE);
  tft.fillCircle(tX, tY, 25, C_RED);
  tft.fillRect(tX - 5, 250 - pxAlto, 10, pxAlto, C_RED);

  // Borramos interior de recuadros
  tft.fillRoundRect(controlX + 2,               controlY + 2, 106, 76, 13, C_BG);
  tft.fillRoundRect(controlX + separacionX + 2,  controlY + 2, 106, 76, 13, C_BG);

  // Marcos
  tft.drawRoundRect(controlX,               controlY, 110, 80, 15, C_PRIMARY);
  tft.drawRoundRect(controlX + separacionX,  controlY, 110, 80, 15, C_PRIMARY);

  // Cabeceras
  tft.setTextSize(1);
  tft.setTextColor(C_WHITE);
  tft.drawCenterString("REAL",     controlX + 55,               controlY + 8);
  tft.drawCenterString("SETPOINT", controlX + separacionX + 55,  controlY + 8);

  // Valores
  tft.setTextSize(2);

  tft.setCursor(controlX + 15, controlY + 40);
  tft.setTextColor(C_WHITE);
  tft.printf("%.1f", tempActual);
  tft.setTextColor(C_ACCENT);
  tft.print(" C");

  tft.setCursor(controlX + separacionX + 15, controlY + 40);
  tft.setTextColor(C_WHITE);
  tft.printf("%.1f", tempConsigna);
  tft.setTextColor(C_ACCENT);
  tft.print(" C");

  // Salida PID (fila extra debajo de los recuadros)
  tft.fillRect(controlX, controlY + 90, 220, 20, C_LTBLUE);
  tft.setTextSize(1);
  tft.setTextColor(C_TEXT_DARK);
  char bufPID[32];
  sprintf(bufPID, "Calefactor: %.1f%%", salida_porcentaje);
  tft.drawCenterString(bufPID, controlX + 110, controlY + 95);
}

// ============================================================
// --- Componentes Gráficos Comunes ---
// ============================================================

void dibujarCabecera(const char* titulo, bool mostrarVolver) {
  tft.fillRect(0, 0, 480, 45, C_BG);
  tft.drawFastHLine(0, 45, 480, C_PRIMARY);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.drawCenterString(titulo, 240, 12);
  if (mostrarVolver) {
    tft.fillRoundRect(8, 6, 110, 32, 6, C_PRIMARY);
    tft.setTextSize(1);
    tft.drawCenterString("< VOLVER", 63, 18);
  }
}

void dibujarBotonCuadrado(int x, int y, const char* txt) {
  tft.fillRoundRect(x, y, 200, 100, 12, C_PRIMARY);
  tft.drawRoundRect(x, y, 200, 100, 12, C_WHITE);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.drawCenterString(txt, x + 100, y + 40);
}

void dibujarIconoBombilla(int x, int y, uint32_t color, bool encendido) {
  tft.fillRect(x - 8, y + 10, 16, 8, 0x7F8C8D);
  tft.fillCircle(x, y - 8, 20, color);
  tft.fillRect(x - 12, y, 24, 10, color);
  tft.drawCircle(x, y - 8, 20, C_TEXT_DARK);
  if (encendido) {
    tft.fillCircle(x - 6, y - 15, 4, C_WHITE);
    for (int i = 0; i < 360; i += 72) {
      float rad = i * 0.01745;
      tft.drawLine(
        x + cos(rad) * 25, (y - 8) + sin(rad) * 25,
        x + cos(rad) * 33, (y - 8) + sin(rad) * 33,
        color
      );
    }
  }
}

void dibujarIconoBalanza(int x, int y) {
  tft.fillRoundRect(x - 50, y + 40, 100, 10, 3, C_TEXT_DARK);
  tft.fillRect(x - 4, y - 50, 8, 90, C_PRIMARY);
  tft.fillRoundRect(x - 70, y - 55, 140, 6, 2, C_ACCENT);
  tft.drawLine(x - 70, y - 55, x - 90, y + 10, C_PRIMARY);
  tft.drawLine(x - 70, y - 55, x - 50, y + 10, C_PRIMARY);
  tft.fillRoundRect(x - 100, y + 10, 60, 6, 2, C_PRIMARY);
  tft.drawLine(x + 70, y - 55, x + 50, y + 10, C_PRIMARY);
  tft.drawLine(x + 70, y - 55, x + 90, y + 10, C_PRIMARY);
  tft.fillRoundRect(x + 40, y + 10, 60, 6, 2, C_PRIMARY);
}

void dibujarIconoGota(int x, int y, uint32_t color) {
  tft.fillCircle(x, y + 10, 22, color);
  for (int i = 0; i <= 30; i++) {
    int ancho = map(i, 0, 30, 0, 22);
    tft.drawFastHLine(x - ancho, y + 10 - i, ancho * 2, color);
  }
  tft.fillCircle(x - 7, y + 5, 5, C_WHITE);
  tft.fillCircle(x - 7, y + 5, 3, color);
  tft.fillCircle(x - 7, y + 5, 2, C_WHITE);
}

void dibujarIconoCO2(int x, int y, uint32_t color) {
  tft.fillCircle(x - 18, y,     20, color);
  tft.fillCircle(x,      y - 8, 25, color);
  tft.fillCircle(x + 20, y,     18, color);
  tft.fillRect(x - 38, y, 78, 25, color);
  tft.fillCircle(x - 18, y,     15, C_LTBLUE);
  tft.fillCircle(x,      y - 8, 19, C_LTBLUE);
  tft.fillCircle(x + 20, y,     13, C_LTBLUE);
  tft.fillRect(x - 32, y, 65, 20, C_LTBLUE);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.drawCenterString("CO2", x, y - 10);
}

void dibujarHospitalGrande(int x, int y) {
  tft.fillRect(x - 40,  y - 90, 80,  110, C_WHITE);
  tft.fillRect(x - 100, y - 40, 60,  60,  C_WHITE);
  tft.fillRect(x + 40,  y - 40, 60,  60,  C_WHITE);
  tft.drawRect(x - 40,  y - 90, 80,  110, C_PRIMARY);
  tft.drawRect(x - 100, y - 40, 60,  60,  C_PRIMARY);
  tft.drawRect(x + 40,  y - 40, 60,  60,  C_PRIMARY);
  for (int r = -80; r < -15; r += 15) {
    tft.fillRect(x - 30, y + r, 15, 8, C_ACCENT);
    tft.fillRect(x + 15, y + r, 15, 8, C_ACCENT);
  }
  for (int r = -30; r < 10; r += 15) {
    tft.fillRect(x - 90, y + r, 12, 8, C_ACCENT);
    tft.fillRect(x - 70, y + r, 12, 8, C_ACCENT);
    tft.fillRect(x + 60, y + r, 12, 8, C_ACCENT);
    tft.fillRect(x + 80, y + r, 12, 8, C_ACCENT);
  }
  tft.fillRect(x - 4,  y - 105, 8,  22, C_RED);
  tft.fillRect(x - 11, y - 97,  22, 8,  C_RED);
  tft.fillRect(x - 15, y + 5,   30, 15, C_TEXT_DARK);
}

void actualizarLecturaPeso() {

  if (millis() - ultimoUpdatePeso > 250) {

    ultimoUpdatePeso = millis();

    // =====================================================
    // SENSOR 1
    // =====================================================

    int32_t raw1 = -HX711_PESO1.readChannelRaw(CHAN_A_GAIN_128);

    float lectura_peso1 = calcularPeso1(raw1);

    peso_filtrado1 =
      alpha * lectura_peso1 +
      (1.0 - alpha) * peso_filtrado1;

    float peso_neto1 = peso_filtrado1 - tara1;

    // =====================================================
    // SENSOR 2
    // =====================================================

    int32_t raw2 = HX711_PESO2.readChannelRaw(CHAN_A_GAIN_128);

    float lectura_peso2 = calcularPeso2(raw2);

    peso_filtrado2 =
      alpha * lectura_peso2 +
      (1.0 - alpha) * peso_filtrado2;

    float peso_neto2 = peso_filtrado2 - tara2;

    // =====================================================
    // PESO TOTAL
    // =====================================================

    peso_total = peso_neto1 + peso_neto2;

    // Conversión a kg
    float peso_kg = peso_total / 1000.0;

    // =====================================================
    // PANTALLA
    // =====================================================

    tft.fillRect(281, 145, 158, 50, C_BG);

    tft.setTextColor(C_WHITE);
    tft.setTextSize(4);

    tft.setCursor(290, 155);

    // Mantener decimales
    tft.printf("%.3f", peso_kg);

    tft.setTextSize(2);
    tft.setTextColor(C_ACCENT);

    tft.drawString("kg", 410, 175);

    // =====================================================
    // DEBUG
    // =====================================================

    Serial.print("Peso total: ");
    Serial.print(peso_kg, 3);
    Serial.println(" kg");
  }
}