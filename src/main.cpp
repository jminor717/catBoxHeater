#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include "SPIFFS.h"
#include "ADCBord.h"
#include "DataBuffer.h"
#include <TempHelp.hpp>
#include <freertos/FreeRTOS.h>
#include "freertos/timers.h"
#include <EEPROM.h>
#include "myConfig.hpp"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "..\lib\DNSServer---esp32\src\DNSServer.h"

#define maxDelay 8 //ms  120⁻¹ longet delay between zcd and on high
#define EEPROM_SIZE 12

WebServer server(80);

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

ADS1256 ads;

const int32_t fullyOnDelay = 0;
const int32_t fullyOffDelay = 7500;
const int32_t CatWeightThreshold = 2000;
/** 
 * 5 deg f ~= 40
 * by 5 deg off target should be full on or off
 * Gain = 100   with a 4000 offset to center the output
*/
const int16_t Gain = 100;
const int16_t Offset = fullyOffDelay / 2;
int32_t FrontHeatDelayUs = fullyOffDelay;
int32_t RearHeatDelayUs = fullyOffDelay;
bool IsOn = false;
bool outputSet = false;
int32_t frontWeight = 0;
int32_t RearWeight = 0;
bool overrideActive = false;
int32_t OverrideActiveForS = 0;

int16_t FW1, FW2, FW3, RW1, RW2, RW3;
int16_t FT, AT;
Intbuffer FW1buffer(20), FW2buffer(20), FW3buffer(20), RW1buffer(20), RW2buffer(20), RW3buffer(20);
uint16_t FW1CalEmpty = 1000, FW2CalEmpty = 1000, FW3CalEmpty = 1000, RW1CalEmpty = 1000, RW2CalEmpty = 1000, RW3CalEmpty = 1000;

Tempsensor FrontTempSense(FrontTemp);
Tempsensor AmbientTempSense(AmbientTemp);
//Tempsensor AmbientTempSense(AmbientTemp);

Tempbuffer FrontTempbuffer(20), AmbientTempbuffer(20);

bool zeroCrossed = false;

#define NUM_TIMERS 3
TimerHandle_t xTimers[NUM_TIMERS];

uint32_t FT_id = 0;
uint32_t AT_id = 1;
uint32_t Calculate_id = 2;

bool measureFrontTemp = false;
bool measureAmbientTemp = false;
bool DoCalculate = false;

//celsius = (float)raw / 16.0;
//fahrenheit = celsius * 1.8 + 32.0;
//((80 -32)/1.8)*16=427
//((60 -32)/1.8)*16=250
//((35 -32)/1.8)*16=27
const int16_t activeTemp = 250;
const int16_t idleTemp = 27;
const int16_t HeaterMaxTemp = 427;

const int8_t GainArrLength = 46;
const int8_t SinGain[GainArrLength] = {
    20, 20, 20, 20, 21, 21, 21,
    22, 23, 23, 24, 25, 26, 27,
    28, 29, 30, 31, 33, 34, 36,
    37, 39, 40, 42, 44, 45, 47,
    49, 51, 53, 55, 57, 59, 61,
    63, 65, 67, 69, 71, 73, 75,
    77, 79, 82, 84};

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void TimerCallback(TimerHandle_t xTimer)
{
    uint32_t Timer_id = (uint32_t)pvTimerGetTimerID(xTimer);
    //vTimerSetTimerID(xTimer, (void *)ulCount);
    if (Timer_id == FT_id)
    {
        measureFrontTemp = true;
    }
    else if (Timer_id == AT_id)
    {
        measureAmbientTemp = true;
    }
    else if (Timer_id == Calculate_id)
    {
        DoCalculate = true;
    }
}

TickType_t measureFrontTempTicks = pdMS_TO_TICKS(1000);
TickType_t measureAmbientTempTicks = pdMS_TO_TICKS(1000);
#define CalculateMs 2000
#define Calculates 2
TickType_t DoCalculateTicks = pdMS_TO_TICKS(CalculateMs);

void StartMyTimer(TimerHandle_t timer)
{
    if (timer == NULL)
    {
        //* The timer was not created.
    }
    else
    {
        if (xTimerStart(timer, 0) != pdPASS)
        {
            //* The timer could not be set into the Active state.
        }
    }
}

void handle_OnConnect()
{
    File file2 = SPIFFS.open("/test.html");
    if (!file2)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    //Serial.println("File Content:");
    int fileSize = file2.available();
    uint8_t fill[fileSize] = {0};
    file2.read(fill, fileSize);
    file2.close();

    String resp = (char *)fill;

    Serial.println("GPIO4 Status: OFF | GPIO5 Status: OFF");
    server.send(200, "text/html", resp.substring(0, fileSize));
    //server.send(200, "text/html", SendHTML(LED1status, LED2status));
}

void overrideON()
{
    overrideActive = true;
    OverrideActiveForS = 3600;
    Serial.println("Override: ON");
    server.send(200, "application/json", "{\"time\":3600000}");
}

void overrideOFF()
{
    overrideActive = false;
    OverrideActiveForS = 0;
    Serial.println("Override: OFF");
    server.send(200, "application/json", "{\"time\":0}");
}

void sendData()
{
    server.send(200, "application/json", "{\"RD\": " + String(RearHeatDelayUs) + ",\"FD\": " + String(FrontHeatDelayUs) + ",\"FW1\": " + String(FW1) + ",\"FW2\":" + String(FW2) + ",\"FW3\":" + String(FW3) + ",\"RW1\":" + String(RW1) + ",\"RW2\":" + String(RW2) + ",\"RW3\":" + String(RW3) + ",\"FT\":" + String(FrontTempbuffer.buffer[FrontTempbuffer.head]) + ",\"RT\":" + String(AmbientTempbuffer.buffer[AmbientTempbuffer.head]) + ",\"FTT\":" + String(FrontTempbuffer.lastUpdated) + ",\"RTT\":" + String(AmbientTempbuffer.lastUpdated) + "}");
}

void handle_NotFound()
{
    server.send(404, "text/plain", "Not found");
}

void SaveCalibration()
{
    EEPROM.write(0, (FW1CalEmpty & 0xff));
    EEPROM.write(1, ((FW1CalEmpty & 0xff00) >> 8));

    EEPROM.write(2, (FW2CalEmpty & 0xff));
    EEPROM.write(3, ((FW2CalEmpty & 0xff00) >> 8));

    EEPROM.write(4, (FW3CalEmpty & 0xff));
    EEPROM.write(5, ((FW3CalEmpty & 0xff00) >> 8));

    EEPROM.write(6, (RW1CalEmpty & 0xff));
    EEPROM.write(7, ((RW1CalEmpty & 0xff00) >> 8));

    EEPROM.write(8, (RW2CalEmpty & 0xff));
    EEPROM.write(9, ((RW2CalEmpty & 0xff00) >> 8));

    EEPROM.write(10, (RW3CalEmpty & 0xff));
    EEPROM.write(11, ((RW3CalEmpty & 0xff00) >> 8));

    EEPROM.commit();
}

void ReadCallibration()
{
    FW1CalEmpty = EEPROM.read(0);
    FW1CalEmpty = FW1CalEmpty | (EEPROM.read(1) << 8);

    FW2CalEmpty = EEPROM.read(2);
    FW2CalEmpty = FW2CalEmpty | (EEPROM.read(3) << 8);

    FW3CalEmpty = EEPROM.read(4);
    FW3CalEmpty = FW3CalEmpty | (EEPROM.read(5) << 8);

    RW1CalEmpty = EEPROM.read(6);
    RW1CalEmpty = RW1CalEmpty | (EEPROM.read(7) << 8);

    RW2CalEmpty = EEPROM.read(8);
    RW2CalEmpty = RW2CalEmpty | (EEPROM.read(9) << 8);

    RW3CalEmpty = EEPROM.read(10);
    RW3CalEmpty = RW3CalEmpty | (EEPROM.read(11) << 8);

    Serial.print("calibration loaded  FW1:");
    Serial.print(FW1CalEmpty);
    Serial.print(" FW2:");
    Serial.print(FW2CalEmpty);
    Serial.print(" FW3:");
    Serial.print(FW3CalEmpty);
    Serial.print(" RW1:");
    Serial.print(RW1CalEmpty);
    Serial.print(" RW2:");
    Serial.print(RW2CalEmpty);
    Serial.print(" RW3:");
    Serial.print(RW3CalEmpty);
    Serial.println();
}

void Calibrate()
{
    FW1CalEmpty = FW1buffer.GetRunningAverage();
    FW2CalEmpty = FW2buffer.GetRunningAverage();
    FW3CalEmpty = FW3buffer.GetRunningAverage();
    RW1CalEmpty = RW1buffer.GetRunningAverage();
    RW2CalEmpty = RW2buffer.GetRunningAverage();
    RW3CalEmpty = RW3buffer.GetRunningAverage();
    SaveCalibration();
    Serial.println("calibrated");
    server.send(200, "application/json", "{\"time\":3600000}");
}

void collectData(void *parameter)
{
    for (;;)
    { // infinite loop
        ads.readInputToAdcValuesArray();
        FW1buffer.push(ads.adcValues[0]);
        FW2buffer.push(ads.adcValues[1]);
        FW3buffer.push(ads.adcValues[2]);
        RW1buffer.push(ads.adcValues[3]);
        RW2buffer.push(ads.adcValues[4]);
        RW3buffer.push(ads.adcValues[5]);
        vTaskDelay(50);
    }
}

void clampValue(int32_t &val, int32_t max, int32_t min)
{
    if (val >= max)
    {
        val = max;
    }
    else if (val <= min)
    {
        val = min;
    }
}

int8_t getGain(int32_t tempDifferential)
{
    //SinGain
    tempDifferential = std::abs(tempDifferential);
    clampValue(tempDifferential, GainArrLength - 1, 0);
    return SinGain[tempDifferential];
}

void RTLoop(void *parameter)
{
    for (;;)
    { // infinite loop
        if (measureFrontTemp)
        {

            measureFrontTemp = false;
            int16_t sssss = FrontTempSense.read();
            int16_t diff = sssss > FT ? sssss - FT : FT - sssss;
            if ((sssss > -1000 && sssss < 2000) && (diff < 50 || FrontTempbuffer.size < FrontTempbuffer.maxsize))
            {
                FrontTempbuffer.push(sssss, xTaskGetTickCount());
            }
        }
        if (measureAmbientTemp)
        {
            measureAmbientTemp = false;
            int16_t sssss = AmbientTempSense.read();
            int16_t diff = sssss > FT ? sssss - FT : FT - sssss;
            if ((sssss > -1000 && sssss < 2000) && (diff < 50 || AmbientTempbuffer.size < AmbientTempbuffer.maxsize))
            {
                AmbientTempbuffer.push(sssss, xTaskGetTickCount());
            }
        }
        if (DoCalculate)
        {
            DoCalculate = false;
            FW1 = FW1CalEmpty - FW1buffer.GetRunningAverage();
            FW2 = FW2CalEmpty - FW2buffer.GetRunningAverage();
            FW3 = FW3CalEmpty - FW3buffer.GetRunningAverage();
            RW1 = RW1CalEmpty - RW1buffer.GetRunningAverage();
            RW2 = RW2CalEmpty - RW2buffer.GetRunningAverage();
            RW3 = RW3CalEmpty - RW3buffer.GetRunningAverage();

            FT = FrontTempbuffer.GetRunningAverage();
            AT = AmbientTempbuffer.GetRunningAverage();
            int16_t FW = ((FW1 + FW2 + FW3) / 3), RW = ((RW1 + RW2 + RW3) / 3);
            bool CatPresent = false;
            int16_t TargetTemp = idleTemp;

            CatPresent = (FW > CatWeightThreshold || RW > CatWeightThreshold);

            if (overrideActive || CatPresent)
            { //cat is present
                OverrideActiveForS -= Calculates;
                if (OverrideActiveForS <= Calculates)
                {
                    overrideActive = false;
                    OverrideActiveForS = 0;
                }
                TargetTemp = activeTemp;
            }

            //temp feedback location
            // 5 deg f ~= 40
            int32_t FeedbackLoopMat = FT - TargetTemp; // inverted feedback because higher delay means lower duty cycle
            int32_t matGain = getGain(FeedbackLoopMat);
            FeedbackLoopMat = (FeedbackLoopMat * matGain) + Offset;

            int32_t FeedbackLoopAmbient = AT - TargetTemp; // inverted feedback because higher delay means lower duty cycle
            int32_t AmbientGain = getGain(FeedbackLoopAmbient);
            FeedbackLoopAmbient = (FeedbackLoopAmbient * AmbientGain) + Offset;

            clampValue(FeedbackLoopAmbient, fullyOffDelay, fullyOnDelay);
            clampValue(FeedbackLoopMat, fullyOffDelay, fullyOnDelay);
            //ideally gain would be variable because a small change close to 50% duty cycle will have a larger impact on the output power than the same change closer to 0 or 100

            if (FT >= HeaterMaxTemp)
            {
                FeedbackLoopMat = 8000;
                FeedbackLoopAmbient = 8000;
            }
            portENTER_CRITICAL_ISR(&timerMux); //if we get an interrupt durring this assignment could be bad
            RearHeatDelayUs = FrontHeatDelayUs = FeedbackLoopMat;
            portEXIT_CRITICAL_ISR(&timerMux);

            TickType_t now = xTaskGetTickCount();

            if (now - FrontTempbuffer.lastUpdated > 30000)
            {
                FrontTempSense = Tempsensor(FrontTemp);
            }
            if (now - AmbientTempbuffer.lastUpdated > 30000)
            {
                AmbientTempSense = Tempsensor(AmbientTemp);
            }
            //clamp output from feedback loop
            //clampValue(RearHeatDelayUs, fullyOffDelay, fullyOnDelay);
            //clampValue(FrontHeatDelayUs, fullyOffDelay, fullyOnDelay);
            Serial.print("   frontUp: ");
            Serial.print(now - FrontTempbuffer.lastUpdated);
            Serial.print("   AmbUp: ");
            Serial.print(now - AmbientTempbuffer.lastUpdated);

            Serial.print("       fw: ");
            Serial.print(FW);
            Serial.print("  rw: ");
            Serial.print(RW);

            Serial.print("   delay: ");
            Serial.println(FrontHeatDelayUs);
        }
    }
}

static void Turn_On_callback(void *arg);
static void Turn_Off_callback(void *arg);
static void nop_callback(void *arg);
esp_timer_handle_t Turn_On_timer;
esp_timer_handle_t Turn_Off_timer;
esp_timer_handle_t Nop_timer;

static void Turn_On_callback(void *arg)
{
    digitalWrite(FrontHeaterCTRL, HIGH);
    digitalWrite(RearHeaterCTRL, HIGH);

    esp_timer_create_args_t Turn_Off_timer_args;
    Turn_Off_timer_args.callback = &Turn_Off_callback;
    Turn_Off_timer_args.name = "TurnOff";
    ESP_ERROR_CHECK(esp_timer_create(&Turn_Off_timer_args, &Turn_Off_timer));
    int32_t onTime = (fullyOffDelay - FrontHeatDelayUs) - 2000;

    clampValue(onTime, 5000, 500);
    ESP_ERROR_CHECK(esp_timer_start_once(Turn_Off_timer, onTime));

    ESP_ERROR_CHECK(esp_timer_delete(Turn_On_timer));
}

static void Turn_Off_callback(void *arg)
{
    digitalWrite(FrontHeaterCTRL, LOW);
    digitalWrite(RearHeaterCTRL, LOW);
    zeroCrossed = false;
    ESP_ERROR_CHECK(esp_timer_delete(Turn_Off_timer));
}

static void nop_callback(void *arg)
{
    zeroCrossed = false;
    ESP_ERROR_CHECK(esp_timer_delete(Nop_timer));
}

void IRAM_ATTR zerocrossCallbackAZ()
{
    //portENTER_CRITICAL_ISR(&timerMux);

    //portEXIT_CRITICAL_ISR(&timerMux);
    //outputSet    IsOn
    if (!zeroCrossed)
    {
        zeroCrossed = true;
        if (FrontHeatDelayUs == fullyOffDelay)
        {
            digitalWrite(FrontHeaterCTRL, LOW);
            digitalWrite(RearHeaterCTRL, LOW);

            esp_timer_create_args_t Nop_timer_args;
            Nop_timer_args.callback = &nop_callback;
            Nop_timer_args.name = "OffNop";
            ESP_ERROR_CHECK(esp_timer_create(&Nop_timer_args, &Nop_timer));
            ESP_ERROR_CHECK(esp_timer_start_once(Nop_timer, 4000)); //debounce timer
        }
        else if (FrontHeatDelayUs == fullyOnDelay)
        {
            digitalWrite(FrontHeaterCTRL, HIGH);
            digitalWrite(RearHeaterCTRL, HIGH);

            esp_timer_create_args_t Nop_timer_args;
            Nop_timer_args.callback = &nop_callback;
            Nop_timer_args.name = "OffNop";
            ESP_ERROR_CHECK(esp_timer_create(&Nop_timer_args, &Nop_timer));
            ESP_ERROR_CHECK(esp_timer_start_once(Nop_timer, 4000));
        }
        else
        {
            esp_timer_create_args_t Turn_On_timer_args;
            Turn_On_timer_args.callback = &Turn_On_callback;
            Turn_On_timer_args.name = "TurnOn";
            ESP_ERROR_CHECK(esp_timer_create(&Turn_On_timer_args, &Turn_On_timer));
            ESP_ERROR_CHECK(esp_timer_start_once(Turn_On_timer, FrontHeatDelayUs));
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    pinMode(zcdPin, INPUT);
    pinMode(FrontHeaterCTRL, OUTPUT);
    pinMode(RearHeaterCTRL, OUTPUT);

    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);

    ads.init(-1, -1, -1, 1000000);
    Serial.println(ads.speedSPI);

    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    ReadCallibration();

    Serial.println("Connecting to ");
    Serial.println(WIFIssid);

    //connect to your local wi-fi network
    //WiFi.begin(WIFIssid, WIFIpassword);
    WiFi.softAP(WIFIapssid, WIFIappassword);

    IPAddress apIP = WiFi.softAPIP();
    //check wi-fi is connected to wi-fi network
    /*  const TickType_t xDelay = 15000 / portTICK_PERIOD_MS;
    TickType_t start = xTaskGetTickCount();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
        TickType_t end = xTaskGetTickCount();
        if (end > start + xDelay)
        {
            ESP.restart();
        }
    } */
    Serial.println("");
    Serial.println("WiFi connected..!");
    Serial.print("Got IP: ");
    Serial.println(apIP);

    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);

    if (!MDNS.begin(WIFI_HOST)) //catbox
    {                           //http://catbox.local
        Serial.println("Error setting up MDNS responder!");
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");

    server.on("/cat", handle_OnConnect);
    server.on("/overrideON", overrideON);
    server.on("/overrideOFF", overrideOFF);
    server.on("/catData", sendData);
    server.on("/calibrate", Calibrate);
    server.onNotFound(handle_NotFound);

    server.begin();

    attachInterrupt(zcdPin, zerocrossCallbackAZ, RISING);

    xTimers[FT_id] = xTimerCreate("MeasureAZTemp", measureFrontTempTicks, pdTRUE, (void *)FT_id, TimerCallback);
    xTimers[AT_id] = xTimerCreate("MeasureELTemp", measureAmbientTempTicks, pdTRUE, (void *)AT_id, TimerCallback);
    xTimers[Calculate_id] = xTimerCreate("MeasureELTemp", DoCalculateTicks, pdTRUE, (void *)Calculate_id, TimerCallback);
    StartMyTimer(xTimers[Calculate_id]);
    delay(333);
    StartMyTimer(xTimers[FT_id]);
    delay(333);
    StartMyTimer(xTimers[AT_id]);

    xTaskCreate(
        collectData,    // Function that should be called
        "collect data", // Name of the task (for debugging)
        1000,           // Stack size (bytes)
        NULL,           // Parameter to pass
        1,              // Task priority
        NULL            // Task handle
    );
    xTaskCreate(
        RTLoop,      // Function that should be called
        "Fast Loop", // Name of the task (for debugging)
        8192,        // Stack size (bytes)
        NULL,        // Parameter to pass
        1,           // Task priority
        NULL         // Task handle
    );
}
void loop()
{
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(50);
}
