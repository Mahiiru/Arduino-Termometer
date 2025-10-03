#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Hash.h>

// ========================================= LCD OLED =============================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// ========================================= SOCKET IO =============================================
const char *SERIAL_NUMBER = ""; // SERIAL NUMBER
int value = 1;
SocketIOclient socketIO;
#define USE_SERIAL Serial

// ========================================= DHT =============================================
#include "DHT.h"
#define DHTPIN D4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);
float humi = 0.0;
float temp = 0.0;

// ========================================= VARIABEL =============================================
//ganti
unsigned long previousMillis = 0;
const long interval = 5000;
int resetButton = D0;

unsigned long previousLcd;
unsigned long intervalLcd = 1000;

bool isConnected = true;
int stateWifi = 0;

unsigned long previousWifi;
unsigned long intervalWifi = 1000;

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void setup()
{
    Serial.begin(9600); // SERIAL
    dht.begin();        // DHT
    delay(10);
    Serial.println('\n');

    // ========================================= LCD OLED =============================================
    Serial.println("OLED intialized");
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

    display.display();
    delay(200);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    delay(200);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("         ");
    display.println("True Color Never Lies");
    display.display();
    delay(1000);
    display.clearDisplay();

    // ========================================= INITIALIZE =============================================
    pinMode(resetButton, INPUT_PULLUP);

    // =================================== WIFI MANAGER =====================================
    WiFiManager wifiManager;
    wifiManager.setAPCallback([&](WiFiManager *wm){
        Serial.println("Entered config mode"); 
        });
    wifiManager.setConfigPortalTimeout(5);
    if (!wifiManager.autoConnect("Termometer_Wifi"))
    {
        Serial.println("Gagal menyambung ke WiFi, melanjutkan tanpa WiFi dan MQTT.");
        isConnected = false;
    }
    else
    {
        Serial.println("Terhubung ke WiFi");
        delay(1000);
        isConnected = true;
    }

    // =================================== SOCKET IO =====================================
    socketIO.begin("");
    socketIO.onEvent(socketIOEvent);
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void loop()
{

    // =================================== SOCKET IO =====================================
    if (WiFi.status() != WL_CONNECTED)
    {
        unsigned long currentWifi = millis();

        if ((currentWifi - previousWifi) > intervalWifi)
        {
            stateWifi = 0;
            WiFi.reconnect();
            previousWifi = currentWifi;
        }
    }
    // Lanjutkan hanya jika WiFi terhubung
    if (WiFi.status() == WL_CONNECTED)
    {

        stateWifi = 1;
        // isConnected = true;
    }

    // =================================== SOCKET IO =====================================
    socketIO.loop();

    // =================================== LCD OLED =====================================
    unsigned long currentLcd = millis();
    if ((currentLcd - previousLcd) > intervalLcd)
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Temp: ");
        display.println(temp);
        display.print("Humi: ");
        display.println(humi);
        if(stateWifi == 0){
            logoWifiCros(120, 10);
        }else {
            logoWifi(120, 10);
        }
        display.display();
        previousLcd = currentLcd;
    }

    // =================================== DHT =====================================
    humi = dht.readHumidity();
    temp = dht.readTemperature();
    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Humi: ");
    Serial.println(humi);

    // =================================== SEND DATA SENSOR =====================================
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        kirimDataSensor();
    }

    // =================================== RESET WIFI =====================================
    if (digitalRead(resetButton) == LOW)
    {
        resetWifi();
    }
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void kirimDataSensor()
{
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    array.add("dataTempHumi");
    JsonObject param1 = array.createNestedObject();
    param1["serial_number"] = SERIAL_NUMBER;
    param1["temp"] = temp;
    param1["humi"] = humi;

    String output;
    serializeJson(doc, output);
    socketIO.sendEVENT(output);
    Serial.print("Data terkirim: ");
    Serial.println(output);
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case sIOtype_DISCONNECT:
        USE_SERIAL.printf("[IOc] Disconnected!\n");
        break;
    case sIOtype_CONNECT:
        USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
        socketIO.send(sIOtype_CONNECT, "/");
        kirimDataSensor();
        break;
    case sIOtype_EVENT:
        USE_SERIAL.printf("[IOc] get event: %s\n", payload);
        break;
    case sIOtype_ACK:
        USE_SERIAL.printf("[IOc] get ack: %u\n", length);
        break;
    case sIOtype_ERROR:
        USE_SERIAL.printf("[IOc] get error: %u\n", length);
        break;
    case sIOtype_BINARY_EVENT:
        USE_SERIAL.printf("[IOc] get binary: %u\n", length);
        break;
    case sIOtype_BINARY_ACK:
        USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
        break;
    }
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void logoWifi(int16_t x, int16_t y)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        display.drawCircle(x, y, 10, WHITE);
        display.drawCircle(x, y, 7, WHITE);
        display.drawCircle(x, y, 4, WHITE);
        display.fillCircle(x, y, 2, WHITE);
    }
    else
    {
        display.drawLine(x - 5, y - 5, x + 5, y + 5, WHITE);
        display.drawLine(x - 5, y + 5, x + 5, y - 5, WHITE);
    }
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void logoWifiCros(int16_t x, int16_t y)
{
    // Gambar simbol WiFi
    display.drawCircle(x, y, 10, WHITE); // Lengkungan luar
    display.drawCircle(x, y, 7, WHITE);  // Lengkungan tengah
    display.drawCircle(x, y, 4, WHITE);  // Lengkungan dalam
    display.fillCircle(x, y, 2, WHITE);  // Titik tengah

    // Gambar tanda silang
    display.drawLine(x - 12, y - 12, x + 12, y + 12, WHITE); // Garis diagonal 1
    display.drawLine(x - 12, y + 12, x + 12, y - 12, WHITE); // Garis diagonal 2
}

//=================================================================================================
//===                                                                                           ===
//=================================================================================================
void resetWifi()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(" Wait.....   ");
    display.println("Reset Wifi");
    display.display();

    Serial.println("Resetting WiFi...");
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    ESP.reset();
}
