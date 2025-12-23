#include "led.h"
#include "PWM.h"
#include "logging.h"
static const char *LED_TAG = "LedService";
static u_int countmes = 0;
void Led::init()
{
    SET_OUTPUT(135);
}

String Led::ledOn()
{
    WRITE(135, 1);
    ESP_LOGV(LED_TAG, "Led On");
    state = 1;
    return "Led On";
}

String Led::ledOn(uint16_t dst)
{
    ESP_LOGV(LED_TAG, "Led On to %X", dst);
    if (dst == LoraMesher::getInstance().getLocalAddress())
        return ledOn();

    DataMessage *msg = getLedMessage(LedCommand::On, dst);
    MessageManager::getInstance().sendMessage(messagePort::LoRaMeshPort, msg);

    delete msg;

    return "Led On";
}

String Led::ledOff()
{
    WRITE(135, 0);
    ESP_LOGV(LED_TAG, "Led Off");
    state = 0;
    return "Led Off";
}

String Led::ledOff(uint16_t dst)
{
    ESP_LOGV(LED_TAG, "Led Off to %X", dst);
    if (dst == LoraMesher::getInstance().getLocalAddress())
        return ledOff();

    DataMessage *msg = getLedMessage(LedCommand::Off, dst);
    MessageManager::getInstance().sendMessage(messagePort::LoRaMeshPort, msg);

    delete msg;

    return "Led Off";
}
void Led::sendstatus()
{
    PWMCommandMessage *message = new PWMCommandMessage();

    message->ledCommand = (LedCommand)state;
    message->duty = 0;
    message->frequency = 0;

    message->appPortDst = appPort::MQTTApp;
    message->appPortSrc = appPort::LedApp;
    message->addrSrc = LoraMesher::getInstance().getLocalAddress();
    message->addrDst = 0;
    message->messageId = countmes++;
    message->messageSize = sizeof(PWMCommandMessage) - sizeof(DataMessageGeneric);
    // Send the message
    MessageManager::getInstance().sendMessage(messagePort::MqttPort, (DataMessage *)message);

    // Delete the message
    delete message;
}
String Led::ledBlink()
{
    if (state == 1)
    {
        ledOff();
        delay(200);
        ledOn();
        delay(200);
        ledOff();
        delay(200);
        ledOn();
    }
    else
    {
        ledOn();
        delay(200);
        ledOff();
        delay(200);
        ledOn();
        delay(200);
        ledOff();
    }
    return "Led Blink";
}
void setPWM(){

}

String Led::getJSON(DataMessage *message)
{
    LedMessage *PWMMessage = (LedMessage *)message;

    StaticJsonDocument<2000> doc;

    JsonObject root = doc.to<JsonObject>();

    PWMMessage->serialize(root);

    String json;
    serializeJson(doc, json);

    return json;
}

DataMessage *Led::getDataMessage(JsonObject data)
{
    PWMCommandMessage *ledMessage = new PWMCommandMessage();

    ledMessage->deserialize(data);

    ledMessage->messageSize = sizeof(PWMCommandMessage) - sizeof(DataMessageGeneric);

    return ((DataMessage *)ledMessage);
}

DataMessage *Led::getLedMessage(LedCommand command, uint16_t dst)
{
    LedMessage *ledMessage = new LedMessage();

    ledMessage->messageSize = sizeof(LedMessage) - sizeof(DataMessageGeneric);

    ledMessage->ledCommand = command;

    ledMessage->appPortSrc = appPort::LedApp;
    ledMessage->appPortDst = appPort::LedApp;

    ledMessage->addrSrc = LoraMesher::getInstance().getLocalAddress();
    ledMessage->addrDst = dst;

    return (DataMessage *)ledMessage;
}

void Led::processReceivedMessage(messagePort port, DataMessage *message)
{
    PWMCommandMessage *ledMessage = (PWMCommandMessage *)message;

    switch (ledMessage->ledCommand)
    {
    case LedCommand::On:
        ledOn();
        break;
    case LedCommand::Off:
        ledOff();
        break;
    case LedCommand::Sendata:
        DBGLN("Status : %d",state);
        sendstatus();
        break;
    case LedCommand::reciver:
        DBGLN("Staus: pin-%d,duty-%d,frequency:%d",ledMessage->pin,ledMessage->duty,ledMessage->frequency);
        pwm.set_pwm(ledMessage->pin,ledMessage->duty,ledMessage->frequency);
    default:
        break;
    }
}
