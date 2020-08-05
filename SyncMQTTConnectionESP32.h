/*
Arduino Library for very simple and basic MQTT connection, SYNCHRONOUS, arduino-esp32

require #include "PubSubClient.h" library https://github.com/knolleary/pubsubclient.git PlatforIO ID_89

require #include "verysimpletimer.h">"  https://github.com/hagre/VerySimpleTimer_Library.git

GPLv3
by hagre 2020
*/

#ifndef syncmqttconnectionesp32_h 
#define syncmqttconnectionesp32_h

#include <Arduino.h>

#include "IPAddress.h" // to use IP type
#include "Client.h" // to relay WIFI (or LAN) client to PubSub library 
#include "PubSubClient.h"

#include "verysimpletimer.h" 

#ifndef MQTT_WAIT_FOR_SERVER_CONNECTION
    #define MQTT_WAIT_FOR_SERVER_CONNECTION 30000 //ms time
#endif
#ifndef MQTT_WAIT_FOR_SERVER_RECONNECTION
    #define MQTT_WAIT_FOR_SERVER_RECONNECTION 15000 //ms time
#endif

//switch ASAP off
//#define DEBUG_MY_MQTT_ENABLED 

#include <HardwareSerial.h>

#ifndef MAX_NUM_OFF_MQTT_SUBSCRIPTION
    #define MAX_NUM_OFF_MQTT_SUBSCRIPTION 20 //config as required
#endif

#ifndef MAX_MQTT_SUBSCRIPTION_TOPIC_LENGTH
    #define MAX_MQTT_SUBSCRIPTION_TOPIC_LENGTH 100 //config as required
#endif


#if defined(ESP8266) || defined(ESP32)
    #include <functional>
    #define MY_MQTT_CALLBACK_SIGNATURE std::function<void(char*, uint8_t*, unsigned int)> mycallback
#else
    #define MY_MQTT_CALLBACK_SIGNATURE void (*mycallback)(char*, uint8_t*, unsigned int)
#endif



class SyncMQTTConnectionESP32
{
private:
    HardwareSerial* _mqttDebugSerial;
    int8_t _MQTTStatus;

    struct SubscriptionBuffer_t {
        bool inUse = false;
        bool toSubscribe = false;
        bool toUnsubscribe = false;
        bool subscribed = false;
        char topic[MAX_MQTT_SUBSCRIPTION_TOPIC_LENGTH];
        uint16_t topicLength;
    } _subscriptionBuffer[MAX_NUM_OFF_MQTT_SUBSCRIPTION];

    const char* _mqttPubSubClientId;
    const char* _mqttUsername;
    const char* _mqttPassword;
    bool _cleanSessionClient;
    bool _withPW;
    bool _withLastWill;

    const char* _mQTTLastWillTopic;
    const char* _mQTTLastWillMessage;
    uint8_t _mQTTLastWillQos;
    bool _mQTTLastWillRetain;

    VerySimpleTimer _MQTTWaitForConnectionTimer;
    VerySimpleTimer _MQTTWaitForReconnectionTimer;

    PubSubClient _mqttPubSubClient;

    void setUnsubscribed(uint8_t num);
    void setAllUnsubscribed();

    bool checkSubscriptionActive(uint8_t num);
    bool manageSubscriptions(bool justSubscribe);

public:
    SyncMQTTConnectionESP32();

    void setMQTTDebugSerial (HardwareSerial* mqttDebugSerial);

    void setMQTTConnection(const char* mqttPubSubClientId, const char* mqttUsername, const char* mqttPassword, bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout);
    void setMQTTConnection(const char* mqttPubSubClientId, const char* mqttUsername, const char* mqttPassword, bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout);
    void setMQTTConnection(const char* mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout);
    void setMQTTConnection(const char* mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout);
    void setMQTTCallback(MY_MQTT_CALLBACK_SIGNATURE);

    void setMQTTLastWill(const char* mQTTLastWillTopic, uint8_t mQTTLastWillQos, bool mQTTLastWillRetain, const char* mQTTLastWillMessage);
    void deleteMQTTLastWill();

    bool checkSubscriptionUsed(uint8_t num);
    bool addSubscriptionToTable(uint8_t num, const char* topic, uint16_t topicLength);
    bool deleteSubscriptioFromTable(uint8_t num);
    char* getTopicFromBuffer(uint8_t num);

    bool publish(const char* topic, const uint8_t * payload, unsigned int plength, boolean retain);
    bool publish(const char* topic, const char* payload, bool retain);
    // bool publish (const char* topic, const uint8_t * payload, unsigned int plength, uint8_t qOS, bool retain); //not supported by PubSubCLient
    int8_t loop(uint32_t millistime, uint8_t lANStatus);
};

#endif