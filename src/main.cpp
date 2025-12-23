#include "lora_common.h"
#include "options.h"
#include "devLED.h"
#include "devWIFI.h"
#include "devButton.h"
#include "devMQTT.h"
#include "SENSOR/devSensor.h"
#include "PWM.h"
#include "FIFO.h"
#if defined(PLATFORM_ESP32)
#include "devScreen.h"
#include "devETH.h"
#include "led/led.h"

Led &led = Led::getInstance();

void initLed()
{
    led.init();
}

#endif
// Manager
#include "messageManager.h"

// LoRaMesh
#include "loraMeshService.h"

static const char *TAG = "Main";
unsigned long rebootTime = 0;
device_affinity_t devices[] = {
    {&RGB_device, 0},
    {&WIFI_device, 0},
    {&Button_device, 0},
    {&Screen_device, 0},
#ifdef GATEWAY
    {&MQTT_device, 0},
#endif
    {&Sensor_dev, 0},
    {&Send_message, 0}
    };

Stream *NodeUSB;
Stream *NodeBackpack;
Stream *Node485;
#define UART_INPUT_BUF_LEN 1024
FIFO<UART_INPUT_BUF_LEN> uartInputBuffer;
/**
 * Target-specific initialization code called early in setup()
 * Setup GPIOs or other hardware, config not yet loaded
 ***/

LoRaMeshService &loraMeshService = LoRaMeshService::getInstance();

DEV_MQTT &mqttService = DEV_MQTT::getInstance();
SensorService &sensorService = SensorService::getInstance();
MessageManager &manager = MessageManager::getInstance();
void initManager()
{
    manager.init();
    ESP_LOGV(TAG, "Manager initialized");
    manager.addMessageService(&loraMeshService);
    ESP_LOGV(TAG, "LoRaMesher service added to manager");
    manager.addMessageService(&mqttService);
    ESP_LOGV(TAG, "MQTT service added to manager");
    manager.addMessageService(&sensorService);
    ESP_LOGV(TAG, "MQTT service added to manager");
    manager.addMessageService(&led);
    ESP_LOGV(TAG, "Led service added to manager");
}

#if defined(PLATFORM_ESP32_S3)
#include "USB.h"
#define USBSerial Serial
#endif

void setupSerial()
{
#if defined(PLATFORM_ESP32)
    Stream *serialPort;
    if (GPIO_PIN_DEBUG_RX != UNDEF_PIN && GPIO_PIN_DEBUG_TX != UNDEF_PIN)
    {
        serialPort = new HardwareSerial(0);
        ((HardwareSerial *)serialPort)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, GPIO_PIN_DEBUG_RX, GPIO_PIN_DEBUG_TX);
    }
    else
    {
        serialPort = new NullStream();
    }
#elif defined(PLATFORM_ESP8266)
    Stream *serialPort;
    if (GPIO_PIN_DEBUG_TX != UNDEF_PIN)
    {
        serialPort = new HardwareSerial(1);
        ((HardwareSerial *)serialPort)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, SERIAL_TX_ONLY, GPIO_PIN_DEBUG_TX);
    }
    else
    {
        serialPort = new NullStream();
    }
#endif
    NodeBackpack = serialPort;
#if defined(PLATFORM_ESP32_S3)
    Serial.begin(115200);
#endif

// Setup NodeUSB
#if defined(PLATFORM_ESP32_S3)
    USBSerial.begin(firmwareOptions.uart_baud);
    NodeUSB = &USBSerial;
#elif defined(PLATFORM_ESP32)
    if (GPIO_PIN_DEBUG_RX == 3 && GPIO_PIN_DEBUG_TX == 1)
    {
        // The backpack is already assigned on UART0 (pins 3, 1)
        // This is also USB on modules that use DIPs
        // Set NodeUSB to TxBackpack so that data goes to the same place
        NodeUSB = NodeBackpack;
    }
    else if (GPIO_PIN_RCSIGNAL_RX == U0RXD_GPIO_NUM && GPIO_PIN_RCSIGNAL_TX == U0TXD_GPIO_NUM)
    {
        // This is an internal module, or an external module configured with a relay.  Do not setup NodeUSB.
        NodeUSB = new NullStream();
    }
    else
    {
        // The backpack is on a separate UART to UART0
        // Set NodeUSB to pins 3, 1 so that we can access NodeUSB and TxBackpack independantly
        NodeUSB = new HardwareSerial(1);
        ((HardwareSerial *)NodeUSB)->begin(firmwareOptions.uart_baud, SERIAL_8N1, 3, 1);
    }
#else
    NodeUSB = new NullStream();
#endif
#ifdef OPT_HAS_RS485
    Stream *serialPort485;
    if (GPIO_PIN_RS485_RX != UNDEF_PIN && GPIO_PIN_RS485_TX != UNDEF_PIN)
    {
        serialPort485 = new HardwareSerial(1);
        ((HardwareSerial *)serialPort485)->begin(115200, SERIAL_8N1, GPIO_PIN_RS485_RX, GPIO_PIN_RS485_TX);
    }
    else
    {
        serialPort485 = new NullStream();
    }
#endif
    Node485 = serialPort485;
}

// static void HandleUARTin()
// {
//     // Read from the USB serial port
//     if (TxUSB->available())
//     {
//         if (firmwareOptions.is_airport)
//         {
//             auto size = std::min(apInputBuffer.free(), (uint16_t)TxUSB->available());
//             if (size > 0)
//             {
//                 uint8_t buf[size];
//                 TxUSB->readBytes(buf, size);
//                 apInputBuffer.lock();
//                 apInputBuffer.pushBytes(buf, size);
//                 apInputBuffer.unlock();
//             }
//         }
//         else
//         {
//             auto size = std::min(uartInputBuffer.free(), (uint16_t)TxUSB->available());
//             if (size > 0)
//             {
//                 uint8_t buf[size];
//                 TxUSB->readBytes(buf, size);
//                 uartInputBuffer.lock();
//                 uartInputBuffer.pushBytes(buf, size);
//                 uartInputBuffer.unlock();

//                 // Lets check if the data is Mav and auto change LinkMode
//                 // Start the hwTimer since the user might be operating the module as a standalone unit without a handset.
//                 if (connectionState == noCrossfire)
//                 {
//                     if (isThisAMavPacket(buf, size))
//                     {
//                         config.SetLinkMode(TX_MAVLINK_MODE);
//                         UARTconnected();
//                     }
//                 }
//             }
//         }
//     }
// }
static void setupTarget()
{
    setupSerial();
    setupTargetCommon();
}
bool setupHardwareFromOptions()
{
    if (!options_init())
    {
        // Register the WiFi with the framework
        NodeBackpack->println("Wifi start");
        static device_affinity_t wifi_device[] = {
            {&WIFI_device, 1}};
        devicesRegister(wifi_device, ARRAY_SIZE(wifi_device));
        devicesInit();

        return false;
    }
    return true;
}
void handleUartMessage()
{
    while (Serial.available())
    {
        // Đọc dữ liệu từ UART đến khi gặp ký tự '\n'
        String message = Serial.readStringUntil('\n');

        // Xóa ký tự xuống dòng hoặc ký tự không mong muốn cuối chuỗi
        message.trim();

        // In dữ liệu nhận được ra Serial Monitor
        Serial.println("Received message:");
        Serial.println(message);

        // Xử lý lệnh thông qua MessageManager
        String executedProgram = MessageManager::getInstance().executeCommand(message);

        // In kết quả thực thi
        Serial.println("Executed program:");
        Serial.println(executedProgram);

        // Gửi phản hồi qua UART
        Serial.println("Sending response:");
        Serial.println(executedProgram);
    }
}
void setup()
{
    if (setupHardwareFromOptions())
    {
        bool init_success;
        init_success = pwm.init_pwm();
        setupTarget();
        devicesRegister(devices, ARRAY_SIZE(devices));
        devicesInit();
        DBGLN("Initialised devices");
        // Initialize Manager
        initManager();

        ESP_LOGV(TAG, "Heap after initManager: %d", ESP.getFreeHeap());
        init_success = loraMeshService.initLoraMesherService();
        if (!init_success)
        {
            setConnectionState(hardwareUndefined);
        }

        ESP_LOGV(TAG, "Heap after initLoRaMesher: %d", ESP.getFreeHeap());
        NodeUSB->print("Commnad:");
        NodeUSB->println(manager.getAvailableCommands());
    }
    else
    {
        // In the failure case we set the logging to the null logger so nothing crashes
        // if it decides to log something
        NodeBackpack = new NullStream();
    }

    devicesStart();
    initLed();
    // pwm.exWrite(135,1);
}

#include "WiFi.h"
void loop()
{
    unsigned long now = millis();
    devicesUpdate(now);
    if (rebootTime != 0 && now > rebootTime)
    {
        ESP.restart();
    }
}
