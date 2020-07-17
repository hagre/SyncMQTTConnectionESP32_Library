/*
Arduino Library for very simple and basic MQTT connection, SYNCHRONOUS, arduino-esp32

require #include "PubSubClient.h" library https://github.com/knolleary/pubsubclient.git PlatforIO ID_89

require #include "verysimpletimer.h">"  https://github.com/hagre/VerySimpleTimer_Library.git

GPLv3
by hagre 2020
*/

#include "SyncMQTTConnectionESP32.h"

#include <Arduino.h>

#include "IPAddress.h"
#include "Client.h"
#include "PubSubClient.h"

#include "verysimpletimer.h"


SyncMQTTConnectionESP32::SyncMQTTConnectionESP32() {

    _MQTTStatus = -5;
    _MQTTWaitForConnectionTimer.setIntervalMs(MQTT_WAIT_FOR_SERVER_CONNECTION);
    _MQTTWaitForReconnectionTimer.setIntervalMs(MQTT_WAIT_FOR_SERVER_RECONNECTION);

    _withPW = false;
    _withLastWill = false;

#ifdef DEBUG_MQTT_ENABLED
    Serial.begin(DEBUG_MQTT_BOUD); //Debug output, usually USB Port;
#endif
}


//withPW and IP
void SyncMQTTConnectionESP32::setMQTTConnection(const char* mqttPubSubClientId, const char* mqttUsername, const char* mqttPassword, bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout) {

    _mqttPubSubClient.setClient(mqttLANClient);
    _mqttPubSubClient.setServer(ip, port);
    _mqttPubSubClient.setBufferSize(mQTTMaxPacketSize);
    _mqttPubSubClient.setKeepAlive(mQTTKeepAlive);
    _mqttPubSubClient.setSocketTimeout(socketTimeout);

    _mqttPubSubClientId = mqttPubSubClientId;
    _mqttUsername = mqttUsername;
    _mqttPassword = mqttPassword;

    _cleanSessionClient = cleanSessionClient;

    _withPW = true;
}


//withPW and url
void SyncMQTTConnectionESP32::setMQTTConnection(const char* mqttPubSubClientId, const char* mqttUsername, const char* mqttPassword, bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout) {

    _mqttPubSubClient.setClient(mqttLANClient);
    _mqttPubSubClient.setServer(domain, port);
    _mqttPubSubClient.setBufferSize(mQTTMaxPacketSize);
    _mqttPubSubClient.setKeepAlive(mQTTKeepAlive);
    _mqttPubSubClient.setSocketTimeout(socketTimeout);

    _mqttPubSubClientId = mqttPubSubClientId;
    _mqttUsername = mqttUsername;
    _mqttPassword = mqttPassword;

    _cleanSessionClient = cleanSessionClient;

    _withPW = true;
}

//withoutPW and IP
void SyncMQTTConnectionESP32::setMQTTConnection(const char* mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, IPAddress ip, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout) {

    _mqttPubSubClient.setClient(mqttLANClient);
    _mqttPubSubClient.setServer(ip, port);
    _mqttPubSubClient.setBufferSize(mQTTMaxPacketSize);
    _mqttPubSubClient.setKeepAlive(mQTTKeepAlive);
    _mqttPubSubClient.setSocketTimeout(socketTimeout);

    _mqttPubSubClientId = mqttPubSubClientId;

    _cleanSessionClient = cleanSessionClient;

    _withPW = false;
}

//withoutPW and url
void SyncMQTTConnectionESP32::setMQTTConnection(const char* mqttPubSubClientId, bool cleanSessionClient, Client& mqttLANClient, const char* domain, uint16_t port, uint16_t mQTTMaxPacketSize, uint16_t mQTTKeepAlive, uint16_t socketTimeout) {

    _mqttPubSubClient.setClient(mqttLANClient);
    _mqttPubSubClient.setServer(domain, port);
    _mqttPubSubClient.setBufferSize(mQTTMaxPacketSize);
    _mqttPubSubClient.setKeepAlive(mQTTKeepAlive);
    _mqttPubSubClient.setSocketTimeout(socketTimeout);

    _mqttPubSubClientId = mqttPubSubClientId;

    _cleanSessionClient = cleanSessionClient;

    _withPW = false;
}


void SyncMQTTConnectionESP32::setMQTTCallback(MY_MQTT_CALLBACK_SIGNATURE) {

    _mqttPubSubClient.setCallback(mycallback);
}

void SyncMQTTConnectionESP32::setMQTTLastWill(const char* mQTTLastWillTopic, uint8_t mQTTLastWillQos, bool mQTTLastWillRetain, const char* mQTTLastWillMessage) {

    _mQTTLastWillTopic = mQTTLastWillTopic;
    _mQTTLastWillMessage = mQTTLastWillMessage;
    _mQTTLastWillQos = mQTTLastWillQos;
    _mQTTLastWillRetain = mQTTLastWillRetain;

    _withLastWill = true;

    if (_MQTTStatus != 5) { //not init phase
        _MQTTStatus = -3; //disconnect ans reconnect
    }

}

void SyncMQTTConnectionESP32::deleteMQTTLastWill() {

    if (_withLastWill) {
        _withLastWill = false;

        if (_MQTTStatus != 5) { //not init phase
            _MQTTStatus = -3; //disconnect ans reconnect
        }
    }
}

bool SyncMQTTConnectionESP32::checkSubscriptionUsed(uint8_t num) {

    if (num <= MAX_NUM_OFF_MQTT_SUBSCRIPTION) { //in range
        return  _subscriptionBuffer[num].inUse;
    }
    else {
        return false;
    }
}

bool SyncMQTTConnectionESP32::checkSubscriptionActive(uint8_t num) {

    return  _subscriptionBuffer[num].subscribed;
}

bool SyncMQTTConnectionESP32::addSubscriptionToTable(uint8_t num, byte* topic, uint16_t topicLength) {

    if (num <= MAX_NUM_OFF_MQTT_SUBSCRIPTION) { //in range
        if (!checkSubscriptionUsed(num)) { //new subscription
            _subscriptionBuffer[num].inUse = true;
            _subscriptionBuffer[num].toUnsubscribe = false;
            _subscriptionBuffer[num].toSubscribe = true; //will be done soon
            _subscriptionBuffer[num].subscribed = false;
            _subscriptionBuffer[num].topicLength = topicLength;
            for (int16_t i = 0; i < topicLength; i++) {
                _subscriptionBuffer[num].topic[i] = topic[i];
            }
            return true;
        }
    }
    return false;
}

bool SyncMQTTConnectionESP32::deleteSubscriptioFromTable(uint8_t num) {

    if (num <= MAX_NUM_OFF_MQTT_SUBSCRIPTION) { //in range
        if (checkSubscriptionUsed(num)) { // already in use
            _subscriptionBuffer[num].inUse = false;
            _subscriptionBuffer[num].toUnsubscribe = true; //will be done soon
            _subscriptionBuffer[num].toSubscribe = false;
            _subscriptionBuffer[num].topicLength = 0;
            for (int16_t i = 0; i < MAX_MQTT_SUBSCRIPTION_TOPIC_LENGTH; i++) {
                _subscriptionBuffer[num].topic[i] = 0;
            };
            return true;
        }
    }
    return false;
}

String SyncMQTTConnectionESP32::getTopicFromBuffer(uint8_t num) {

    String outputString = "";
    for (uint16_t i = 0; i < _subscriptionBuffer[num].topicLength; i++) {
        outputString = outputString + _subscriptionBuffer[num].topic[i];
    }
    return outputString;
}

void SyncMQTTConnectionESP32::setUnsubscribed(uint8_t num) {

    _subscriptionBuffer[num].toUnsubscribe = false;
    _subscriptionBuffer[num].toSubscribe = true; //will be done soon
    _subscriptionBuffer[num].subscribed = false;
}

void SyncMQTTConnectionESP32::setAllUnsubscribed() {

    for (uint8_t i = 0; i < MAX_NUM_OFF_MQTT_SUBSCRIPTION; i++) {
        setUnsubscribed(i);
    }
}

bool SyncMQTTConnectionESP32::manageSubscriptions(bool justSubscribe = false) {

    uint8_t doneSubs = 0;
    uint8_t requestetedSubs = 0;

    uint8_t doneUnSubs = 0;
    uint8_t requestetedUnSubs = 0;

    for (uint8_t i = 0; i < MAX_NUM_OFF_MQTT_SUBSCRIPTION; i++) {
        if (checkSubscriptionUsed(i)) {
            if (_subscriptionBuffer[i].toSubscribe && !_subscriptionBuffer[i].toUnsubscribe) {
                requestetedSubs++;
                const char* pTopic = _subscriptionBuffer[i].topic;
                if (_mqttPubSubClient.subscribe(pTopic)) {
                    _subscriptionBuffer[i].subscribed = true;
                    _subscriptionBuffer[i].toSubscribe = false;
                    doneSubs++;
                }
                else {
                    _subscriptionBuffer[i].subscribed = false; //is already
                }
            }
            else if (_subscriptionBuffer[i].toUnsubscribe && justSubscribe) {
                requestetedUnSubs++;
                const char* pTopic = _subscriptionBuffer[i].topic;
                if (_mqttPubSubClient.unsubscribe(pTopic)) {
                    doneUnSubs++;
                    _subscriptionBuffer[i].subscribed = false;
                    _subscriptionBuffer[i].toUnsubscribe = false;
                    _subscriptionBuffer[i].inUse = false;
                }
            }

        }
    }

    if (doneSubs == requestetedSubs && doneUnSubs == requestetedUnSubs) {
        return true;
    }

    return false;
}


bool SyncMQTTConnectionESP32::publish(const char* topic, const uint8_t* payload, unsigned int plength, bool retain) {

    return _mqttPubSubClient.publish(topic, payload, plength, retain);
}

/*
not supported by PubSubCLient
bool SyncMQTTConnectionESP32::publish(const char* topic, const uint8_t *payload, unsigned int plength, uint8_t qOS, bool retain){

    return _mqttPubSubClient.publish(topic, payload, plength, qOS, retain);
}
*/

int8_t SyncMQTTConnectionESP32::loop(uint32_t millistime, uint8_t lANStatus) {

    if (lANStatus == -2) { // LAN is just disconnected 
        if (_MQTTStatus > -2) { // MQTT better than disconnected and LAN just disconnected 
            _MQTTStatus = -3;
#ifdef DEBUG_MQTT_ENABLED 
            Serial.println(" LAN for MQTT just disconnected"); //disconnect -> MQTT stop ");
#endif
        }
    }
    else if (lANStatus == 3) { // LAN is connected look for more
        if (_MQTTStatus == -5) { // MQTT init
            _MQTTStatus = 0; //MQTT disconnected 
#ifdef DEBUG_MQTT_ENABLED 
            Serial.println(" MQTT init loop");
#endif
        }
        else if (_MQTTStatus == -3) { // LAN just disconnected
            _mqttPubSubClient.disconnect();
            setAllUnsubscribed();
            _MQTTStatus = -2; //MQTT just disconnected
#ifdef DEBUG_MQTT_ENABLED 
            Serial.println(" MQTT stop - LAN connection lost");
#endif
        }
        else if (_MQTTStatus == -2) { // MQTT just disconnected
            _MQTTWaitForReconnectionTimer.resetTimingNow(millistime);
            _MQTTStatus = -1; //MQTT wait to reconnect
#ifdef DEBUG_MQTT_ENABLED 
            Serial.println(" MQTT just disconnected");
#endif
        }
        else if (_MQTTStatus == -1) { //MQTT wait to reconnect
            if (_MQTTWaitForReconnectionTimer.getStatus(millistime) >= 0) {
                _MQTTStatus = 0; //MQTT disconncted
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" MQTT time to reconnect ");
#endif
            }
        }
        else if (_MQTTStatus == 0) { //MQTT disconnected 
            _MQTTWaitForConnectionTimer.resetTimingNow(millistime);
            if (_withPW) {
#ifdef DEBUG_MQTT_ENABLED 
                Serial.print(" Connecting MQTT with PW");
#endif
                if (_withLastWill) {
                    _mqttPubSubClient.connect(_mqttPubSubClientId, _mqttUsername, _mqttPassword, _mQTTLastWillTopic, _mQTTLastWillQos, _mQTTLastWillRetain, _mQTTLastWillMessage, _cleanSessionClient);
#ifdef DEBUG_MQTT_ENABLED 
                    Serial.println(" and with LastWill");
#endif
                }
                else {
                    _mqttPubSubClient.connect(_mqttPubSubClientId, _mqttUsername, _mqttPassword);
#ifdef DEBUG_MQTT_ENABLED 
                    Serial.println(" and without LastWill");
#endif
                }
            }
            else {
#ifdef DEBUG_MQTT_ENABLED 
                Serial.print(" Connecting MQTT without PW");
#endif
                if (_withLastWill) {
                    _mqttPubSubClient.connect(_mqttPubSubClientId, _mQTTLastWillTopic, _mQTTLastWillQos, _mQTTLastWillRetain, _mQTTLastWillMessage);
#ifdef DEBUG_MQTT_ENABLED 
                    Serial.println(" and with LastWill");
#endif
                }
                else {
                    _mqttPubSubClient.connect(_mqttPubSubClientId);
#ifdef DEBUG_MQTT_ENABLED 
                    Serial.println(" and without LastWill");
#endif
                }
            }
            _MQTTStatus = 1; //MQTT connecting
        }
        else if (_MQTTStatus == 1) {
            if (_mqttPubSubClient.connected()) {
                _MQTTStatus = 2; //MQTT just connected
                //boot/new connect = true; ----------------------------------------------------------------------------------------------------BOOT MSG BUFFER ----------------------------------------------------------------------------------
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" MQTT just connected");
#endif
            }
            else if (_MQTTWaitForConnectionTimer.getStatus(millistime) >= 0) {
                _MQTTStatus = -2; // MQTT just disconnected
            }
        }
        else if (_MQTTStatus == 2) { //MQTT just connected
            _MQTTStatus = 3; //subscribing
            if (!_mqttPubSubClient.loop()) {
                _MQTTStatus = -2; // MQTT just disconnected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" MQTT connecting problem in Loop");
#endif
            }
        }
        else if (_MQTTStatus == 3) { //subscribing
            if (!_mqttPubSubClient.loop()) {
                _MQTTStatus = -2; // MQTT just disconnected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" MQTT connecting problem in Loop during subscribing");
#endif
            }
            if (manageSubscriptions(true)) { //just all subscriptions worked
                _MQTTStatus = 4; //subscribed and connected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" Subscribed to all topics ");
#endif
            }
            else {
                _MQTTStatus = -3; //Subscription did not work, so goto LAN just disconnected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" Subscription did not work ");
#endif
            }
        }
        else if (_MQTTStatus == 4) {//subscribed and connected
            if (!_mqttPubSubClient.loop()) {
                _MQTTStatus = -2; // MQTT just disconnected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" MQTT connecting problem in during normal LOOP");
#endif
            }
            if (!manageSubscriptions()) { //problem with subscribing or unsubscribing of changed requests in the buffer
                _MQTTStatus = -3; //Sub- and Unsub-scription did not work, so goto LAN just disconnected
#ifdef DEBUG_MQTT_ENABLED 
                Serial.println(" Sub- and Unsub-scription did not work ");
#endif
            }

#ifdef DEBUG_MQTT_ENABLED 
            //Serial.println ("MQTT Loop duty ");
#endif 
        }
    }

    return _MQTTStatus;
}


