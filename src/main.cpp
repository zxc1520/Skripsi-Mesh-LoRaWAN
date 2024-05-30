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

#include <AsyncMqttClient.h>
#include <WiFi.h>
#include "credentials/MqttKeys.h"

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

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimmer;
TimerHandle_t wifiReconnecTimer;

RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime rtc;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

LoraMesher &radio = LoraMesher::getInstance();
DHTSensor dhtData(DHT_PIN, DHT11);
HCSR04Sensor distances(ECHO_PIN, TRIG_PIN);
// MQTTService mqtt(mqttClient, mqttReconnectTimmer, wifiReconnecTimer);

uint32_t dataCounter = 0;

JsonDocument doc;

unsigned long previousMillis = 0;
const long interval = 25000;

String datas;
String masterDatas;

int ldrData;
float tempData;
float humidData;
int distData;
String srcData;
String nodeTimestampData;
int8_t rssiData;
int8_t snrData;

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
    char sourceNodeDateString[26];

    snprintf_P(receiverDateString,
               countof(receiverDateString),
               PSTR("%02u-%02u-%02u %02u:%02u:%02u"),
               receiverDate.Year(),
               receiverDate.Month(),
               receiverDate.Day(),
               receiverDate.Hour(),
               receiverDate.Minute(),
               receiverDate.Second());

    snprintf_P(sourceNodeDateString,
               countof(receiverDateString),
               PSTR("%02u-%02u-%02u %02u:%02u:%02u"),
               receiverDate.Year(),
               receiverDate.Month(),
               receiverDate.Day(),
               receiverDate.Hour(),
               receiverDate.Minute(),
               receiverDate.Second());

    data.arrivedTimestamp = receiverDateString;

    data.nodeTimestamp = sourceNodeDateString;

    doc["ldr"] = data.ldr;
    doc["humid"] = data.humid;
    doc["temp"] = data.temp;
    doc["distance"] = data.cm;
    doc["address_origin"] = data.src;
    doc["node_timestamp"] = data.nodeTimestamp;
    doc["arrived_timestamp"] = data.arrivedTimestamp;
    doc["rssi"] = data.rssi;
    doc["snr"] = data.snr;
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
        while (radio.getReceivedQueueSize() > 0)
        {

            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            // Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();

            // Print the data packet
            printDataPacket(packet);

            // Sending payload over mqtt
            serializeJson(doc, datas);

            // Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
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

void sendLoRaMessage()
{
    Serial.printf("Send packet %d\n", dataCounter);

    int nilaiSensor = analogRead(LIGHT_DO);
    ldrData = nilaiSensor;

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
        humidData = humid;
        tempData = temp;
    }

    int dist = distances.listen();

    if (dist < 0)
    {
        Serial.print("Failed to attempt calculation !");
    }
    else
    {
        distData = dist;
    }

    char addrStr[15];
    int n = snprintf(addrStr, 15, "%X", radio.getLocalAddress());

    addrStr[n] = '\0';
    srcData = addrStr;

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

    nodeTimestampData = dateString;

    rssiData = radio.getLoraRssi();

    snrData = radio.getLoraSnr();

    doc["ldr"] = ldrData;
    doc["humid"] = humidData;
    doc["temp"] = tempData;
    doc["distance"] = distData;
    doc["address_origin"] = srcData;
    doc["node_timestamp"] = nodeTimestampData;
    doc["arrived_timestamp"] = "n/a";
    doc["rssi"] = rssiData;
    doc["snr"] = snrData;

    doc.shrinkToFit();
    serializeJson(doc, masterDatas);

    // Wait 20 seconds to send the next packet
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    delay(5000);
}

// TaskHandle_t sendLoRaMessage_Handle = NULL;

// void createSendMessage()
// {

//     BaseType_t res = xTaskCreate(
//         sendLoRaMessage,
//         "Send a LoRa Message Routine",
//         4098,
//         (void *)1,
//         1,
//         &sendLoRaMessage_Handle);
//     if (res != pdPASS)
//     {
//         /* code */
//         Serial.printf("Task creation gave error: %d\n");
//         vTaskDelete(sendLoRaMessage_Handle);
//     }
// }

void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void disconnectWifi()
{
    Serial.println("Disconnecting WiFi...");
    WiFi.disconnect();
}

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("WiFi Status: ");
        Serial.println(WiFi.status());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimmer, 0);
        xTimerStart(wifiReconnecTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");
    Serial.print("Session present: ");
    Serial.println(sessionPresent);

    uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_TOPIC, 0);
    Serial.println("Subscribing at QoS 0");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");
    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimmer, 0);
    }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("Subscribe Acked !");
    Serial.print(" packetId: ");
    Serial.println(packetId);
    Serial.print(" qos: ");
    Serial.println(qos);
}

void onMqttUnSubscribe(uint16_t packetId)
{
    Serial.println("Unsubscribe Acknowledged");
    Serial.print(" packetId: ");
    Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId)
{
    Serial.print("Publish acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}

void onMqttMessage(
    char *topic,
    char *payload,
    AsyncMqttClientMessageProperties prop,
    size_t len,
    size_t index,
    size_t total)
{
    String messageTemp;
    for (int i = 0; i < len; i++)
    {
        /* code */
        messageTemp += (char)payload[i];
    }

    if (strcmp(topic, MQTT_SUB_TOPIC) == 0)
    {
        /* code */
        Serial.println("True");
    }

    if (messageTemp == "SEND_DATA")
    {
        /* code */
        // createSendMessage();
        Serial.println("Sensor Data Begin !");
    }

    Serial.println("Publish received!");
    Serial.print(" message: ");
    Serial.println(messageTemp);
    Serial.print(" topic: ");
    Serial.println(topic);
}

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    Rtc.Begin();

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); // setup pin as output for indicator LED
    led_Flash(2, 125);          // two quick LED flashes to indicate program start
    setupLoraMesher();

    // createSendMessage();

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

    mqttReconnectTimmer = xTimerCreate(
        "mqttTimer",
        pdMS_TO_TICKS(2000),
        pdFALSE,
        (void *)0,
        reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

    wifiReconnecTimer = xTimerCreate(
        "wifiTimer",
        pdMS_TO_TICKS(2000),
        pdFALSE,
        (void *)0,
        reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
    connectToWifi();
}

void loop()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
        /* code */
        previousMillis = currentMillis;

        sendLoRaMessage();
        // Publishing slave's node data
        uint16_t packetIdPubData = mqttClient.publish(
            MQTT_PUB_TOPIC,
            1,
            true,
            datas.c_str());
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i, from node: %d", MQTT_PUB_TOPIC, packetIdPubData, radio.getLocalAddress());

        // Publishing master's node data
        uint16_t packetIdMaster = mqttClient.publish(
            MQTT_MASTER_PUB_TOPIC,
            1,
            true,
            String(masterDatas).c_str());
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i, from node: %d", MQTT_MASTER_PUB_TOPIC, packetIdMaster, radio.getLocalAddress());
    }
}