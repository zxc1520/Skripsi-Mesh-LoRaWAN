#include <Arduino.h>
#include "LoraMesher.h"
#include <LoRa.h>
#include <Adafruit_Sensor.h>

#include "RadioLib.h"

#include <ArduinoJson.h>

#include "sensors/DHTSensor.h"
#include "sensors/HCSR04Sensor.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <RtcDS3231.h>

// Using LILYGO TTGO T-BEAM v1.1
#define BOARD_LED 25
#define LED_ON LOW
#define LED_OFF HIGH

// DHT Pin
#define DHT_PIN 15

// Light Sense Pin
#define LIGHT_DO 36

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_DS3231_RTC 0xd0

#define BUTTON_PIN 34
#define SHORT_PRESS 300
#define LONG_PRESS 1000

int lastState = LOW;
int currentState;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime rtc;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

LoraMesher &radio = LoraMesher::getInstance();
DHTSensor dhtData(DHT_PIN, DHT11);
HCSR04Sensor distances(ECHO_PIN, TRIG_PIN);

uint32_t dataCounter = 0;

JsonDocument doc;

struct dataPacket
{
    int ldr;
    float temp;
    float humid;
    int cm;
    String src;
    String nodeTimestamp;
    String arrivedTimestamp;
    int8_t rssi;
    int8_t snr;
};

dataPacket *sensorsPacket = new dataPacket;

// Led flash
void led_Flash(uint16_t flashes, uint16_t delaymS)
{
    uint16_t index;
    for (index = 1; index <= flashes; index++)
    {
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
    }
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data)
{

    RtcDateTime receiverDate = Rtc.GetDateTime();

    char receiverDateString[26];

    snprintf_P(receiverDateString,
               countof(receiverDateString),
               PSTR("%02u-%02u-%02u %02u:%02u:%02u"),
               receiverDate.Year(),
               receiverDate.Month(),
               receiverDate.Day(),
               receiverDate.Hour(),
               receiverDate.Minute(),
               receiverDate.Second());

    sensorsPacket->arrivedTimestamp = receiverDateString;

    doc["ldr"] = sensorsPacket->ldr;
    doc["humid"] = sensorsPacket->humid;
    doc["temp"] = sensorsPacket->temp;
    doc["distance"] = sensorsPacket->cm;
    doc["address_origin"] = sensorsPacket->src;
    doc["node_timestamp"] = sensorsPacket->nodeTimestamp;
    doc["arrived_timestamp"] = sensorsPacket->arrivedTimestamp;
    doc["rssi"] = sensorsPacket->rssi;
    doc["snr"] = sensorsPacket->snr;

    doc.shrinkToFit();
    serializeJsonPretty(doc, Serial);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket> *packet)
{
    Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);

    // Get the payload to iterate through it
    dataPacket *dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();

    for (size_t i = 0; i < payloadLength; i++)
    {
        // Print the packet
        printPacket(dPacket[i]);
    }
}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void *)
{
    for (;;)
    {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(1, 100); // one quick LED flashes to indicate a packet has arrived

        // Iterate through all the packets inside the Received User Packets Queue
        char addrStr[15];
        int n = snprintf(addrStr, 15, "%X", radio.getLocalAddress());

        addrStr[n] = '\0';

        // Get the first element inside the Received User Packets Queue
        AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();

        while (radio.getReceivedQueueSize() > 0)
        {
            if (addrStr != "85CC")
            {
                /* code */
                radio.createPacketAndSend(BROADCAST_ADDR, packet->payload, 1);
            }
            else
            {
                Serial.println("ReceivedUserData_TaskHandle notify received");
                Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

                tone(4, 1000);
                delay(100);
                noTone(4);
                delay(100);

                // Print the data packet
                printDataPacket(packet);

                // Delete the packet when used. It is very important to call this function to release the memory of the packet.
                radio.deletePacket(packet);
            }
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages()
{
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void *)1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS)
    {
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    }

    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
}

/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher()
{
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();
    config.loraCs = 18;
    config.loraRst = 23;
    config.loraIo1 = 26;

    config.freq = 915.0;
    config.module = LoraMesher::LoraModules::SX1276_MOD;

    // Init the loramesher with a processReceivedPackets function
    radio.begin(config);

    // Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    // Start LoRaMesher
    radio.start();

    Serial.println("Lora initialized");
}

void printDateTime(const RtcDateTime &date)
{
    char dateString[26];

    snprintf_P(dateString,
               countof(dateString),
               PSTR("%02u-%02u-%02u %02u:%02u:%02u"),
               date.Year(),
               date.Month(),
               date.Day(),
               date.Hour(),
               date.Minute(),
               date.Second());
}

bool wasError(const char *errorTopic = "")
{
    uint8_t error = Rtc.LastError();
    if (error != 0)
    {
        // we have a communications error
        // see https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
        // for what the number means
        Serial.print("[");
        Serial.print(errorTopic);
        Serial.print("] WIRE communications error (");
        Serial.print(error);
        Serial.print(") : ");

        switch (error)
        {
        case Rtc_Wire_Error_None:
            Serial.println("(none?!)");
            break;
        case Rtc_Wire_Error_TxBufferOverflow:
            Serial.println("transmit buffer overflow");
            break;
        case Rtc_Wire_Error_NoAddressableDevice:
            Serial.println("no device responded");
            break;
        case Rtc_Wire_Error_UnsupportedRequest:
            Serial.println("device doesn't support request");
            break;
        case Rtc_Wire_Error_Unspecific:
            Serial.println("unspecified error");
            break;
        case Rtc_Wire_Error_CommunicationTimeout:
            Serial.println("communications timed out");
            break;
        }
        return true;
    }
    return false;
}

void sendLoRaMessage(void *)
{
    for (;;)
    {
        Serial.printf("Send packet %d\n", dataCounter);

        int nilaiSensor = analogRead(LIGHT_DO);
        sensorsPacket->ldr = nilaiSensor;

        float humid = dhtData.listen();
        float temp = dhtData.temperature();

        if (isnan(humid) || isnan(temp))
        {
            /* code */
            Serial.print("Failed to load sensor");
        }
        else
        {
            /* code */
            sensorsPacket->humid = humid;
            sensorsPacket->temp = temp;
        }

        int dist = distances.listen();

        if (dist < 0)
        {
            Serial.print("Failed to attempt calculation !");
        }
        else
        {
            sensorsPacket->cm = dist;
        }

        // AppPacket<dataPacket>* dp;
        char addrStr[15];
        int n = snprintf(addrStr, 15, "%X", radio.getLocalAddress());

        addrStr[n] = '\0';
        sensorsPacket->src = addrStr;

        if (!Rtc.IsDateTimeValid())
        {
            if (!wasError("loop IsDateTimeValid"))
            {
                Serial.println("RTC lost confidence in the DateTime!");
            }
        }

        RtcDateTime date = Rtc.GetDateTime();
        if (!wasError("loop GetDateTime"))
        {
            printDateTime(date);
            Serial.println();
        }

        // uint32_t unixTime = now.Unix32Time();
        char dateString[26];

        snprintf_P(dateString,
                   countof(dateString),
                   PSTR("%02u-%02u-%02u %02u:%02u:%02u"),
                   date.Year(),
                   date.Month(),
                   date.Day(),
                   date.Hour(),
                   date.Minute(),
                   date.Second());

        sensorsPacket->nodeTimestamp = dateString;

        sensorsPacket->rssi = radio.getLoraRssi();

        sensorsPacket->snr = radio.getLoraSnr();

        // Create packet and send it.
        radio.createPacketAndSend(BROADCAST_ADDR, sensorsPacket, 1);

        // Wait 20 seconds to send the next packet
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

TaskHandle_t sendLoRaMessage_Handle = NULL;

void createSendMessage()
{

    BaseType_t res = xTaskCreate(
        sendLoRaMessage,
        "Send a LoRa Message Routine",
        4098,
        (void *)1,
        1,
        &sendLoRaMessage_Handle);
    if (res != pdPASS)
    {
        /* code */
        Serial.printf("Task creation gave error: %d\n");
        vTaskDelete(sendLoRaMessage_Handle);
    }
}

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    Rtc.Begin();

    pinMode(4, OUTPUT);

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); // setup pin as output for indicator LED
    led_Flash(2, 125);          // two quick LED flashes to indicate program start
    setupLoraMesher();

    createSendMessage();

#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!Rtc.IsDateTimeValid())
    {
        /* code */
        if (!wasError("setup IsDateTimeValid"))
        {
            /* code */
            Serial.println("RTC lost confidence in the DateTime!");

            Rtc.SetDateTime(compiled);
        }
    }

    if (!Rtc.GetIsRunning())
    {
        if (!wasError("setup GetIsRunning"))
        {
            Serial.println("RTC was not actively running, starting now");
            Rtc.SetIsRunning(true);
        }
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (!wasError("setup GetDateTime"))
    {
        if (now < compiled)
        {
            Serial.println("RTC is older than compile time, updating DateTime");
            Rtc.SetDateTime(compiled);
        }
        else if (now > compiled)
        {
            Serial.println("RTC is newer than compile time, this is expected");
        }
        else if (now == compiled)
        {
            Serial.println("RTC is the same as compile time, while not expected all is still fine");
        }
    }

    Rtc.Enable32kHzPin(false);
    wasError("setup Enable32kHzPin");
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
    wasError("setup SetSquareWavePin");

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    dhtData.setup();
    distances.setup();

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    delay(2000);
    display.clearDisplay();

    char addrStr[15];
    snprintf(addrStr, 15, "Id: %X\r\n", radio.getLocalAddress());

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    // Display static text
    display.println(addrStr);
    display.display();
}

void loop()
{
}