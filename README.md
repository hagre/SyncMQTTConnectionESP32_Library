# SyncMQTTConnectionESP32_Library

Arduino Library for very simple and basic MQTT connection, SYNCHRONOUS, arduino-esp32, based on PubSubClient

require #include "PubSubClient.h" library https://github.com/knolleary/pubsubclient.git PlatformIO ID_89

require #include "verysimpletimer.h"  https://github.com/hagre/VerySimpleTimer_Library.git


# Connection to MQTT broker  STATUS:

 -5 MQTT init status, 
 
 -3 LAN just disconnected, disconnect MQTT ans set all buffers to unsubscribe
 
 -2 MQTT just disconnected,
 
 -1 wait to reconnect MQTT, 
 
 0 disconnected, start connection with or without Password afer waiting periode
 
 1 connecting, 
 
 2 just connected, 
 
 3 subscribing, all preselected topics (according buffer)
 
 4 subscribed and connected, re-check update required for subscription or unsubscription (according buffer)


# public Methods:

    SyncMQTTConnectionESP32();
    
    void setMQTTDebugSerial (HardwareSerial* mqttDebugSerial);

    void setMQTTConnection (const char *mqttPubSubClientId, const char *mqttUsername, const char *mqttPassword,  bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout );

    void setMQTTConnection (const char *mqttPubSubClientId, const char *mqttUsername, const char *mqttPassword,  bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout );

    void setMQTTConnection (const char *mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout );

    void setMQTTConnection (const char *mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout );

    void setMQTTCallback (MY_MQTT_CALLBACK_SIGNATURE);

    void setMQTTLastWill (const char* mQTTLastWillTopic, uint8_t mQTTLastWillQos, bool mQTTLastWillRetain, const char* mQTTLastWillMessage);

    void deleteMQTTLastWill ();
    
    bool checkSubscriptionUsed (uint8_t num);

    bool addSubscriptionToTable (uint8_t num, byte* topic , uint16_t topicLength);

    bool deleteSubscriptioFromTable (uint8_t num);
    
    bool publish (const char* topic, const uint8_t * payload, unsigned int plength, bool retained);
    
    int8_t loop (uint32_t millistime, uint8_t lANStatus);



GPLv3
by hagre 2020
