
// wifi mqtt json

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>

// WiFi
const char *ssid = "scxazy";       // Enter your WiFi name
const char *password = "cx020629"; // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "114.55.251.57";
const char *topic = "scx/test";
const char *mqtt_username = "scx";
const char *mqtt_password = "test";
const int mqtt_port = 1883;

// mqtt json 
StaticJsonDocument<200> doc;


// TDS 所需要的 参数
float TU = 0.0;
float TU_value = 0.0;
float TU_calibration = 0.0;
float temp_data = 25.0;
// 这个温度应该是要被读取的 但是默认现在是室温环境好了
float K_Value = 3347.19;

// PH 所需要的参数
#define SensorPin 36            //pH meter Analog output to Arduino Analog Input 2
#define Offset 12.88           //deviation compensate
// #define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;  



WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
    // Set software serial baud to 115200;
    Serial.begin(115200);
    // connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    // connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected())
    {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
        {
            Serial.println("Public emqx mqtt broker connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // publish and subscribe
    client.publish(topic, "Hi EMQ X I'm ESP32 ^^");
    
    // 先不让水质检测器订阅了
    // client.subscribe(topic);
}

// 回调代码
void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void loop()
{
    client.loop();
    int sensorValue = analogRead(36);        // read the input on analog pin 0:
    float TU = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    TU_calibration = -0.0192 * (temp_data - 25) + TU;
    TU_value = -865.68 * TU_calibration + K_Value;
    Serial.print("SensorValue ");
    Serial.println(sensorValue);
    Serial.print("TU ");
    Serial.println(TU);
    if (TU_value <= 0)
    {
        TU_value = 0;
    }
    if (TU_value >= 3000)
    {
        TU_value = 3000;
    }

    // use json to transmit mqtt message
    // String output;
    float TDS = TU_value;
    // 这里把 ph 写死先
    float PH = 7.0;



    // ph 数据 获取
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue, voltage;
    if (millis() - samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++] = analogRead(SensorPin);
        if (pHArrayIndex == ArrayLenth)
            pHArrayIndex = 0;
        voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
        pHValue = -0.6897 * voltage + Offset;
        if (pHValue <= 0.0)
        {
            pHValue = 0.0;
        }
        if (pHValue > 14.0)
        {
            pHValue = 14.0;
        }
        samplingTime = millis();
    }
    // 线性拟合了 pHValue 

    PH = pHValue;



    // json 数据 发送 mqtt
    doc["TDS"] = TDS;
    doc["PH"] = PH;

    // 串口打印 TDS
    Serial.print("TU Value:");
    Serial.println(TU_value); // print out the value you read:
    // 串口打印PH 
    // Serial.print("PH:");
    // Serial.println(PH);
    if (millis() - printTime > printInterval) // Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    {
        Serial.print("Voltage:");
        Serial.print(voltage, 2);
        Serial.print("    pH value: ");
        Serial.println(pHValue, 2);
        // digitalWrite(LED,digitalRead(LED)^1);
        printTime = millis();
    }

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish(topic, buffer, n);

    // String TUString = String(TU_value);
    // client.publish(topic,TUString);
    delay(1000);
    
}


// PH 参数处理
double avergearray(int *arr, int number)
{
    int i;
    int max, min;
    double avg;
    long amount = 0;
    if (number <= 0)
    {
        Serial.println("Error number for the array to avraging!/n");
        return 0;
    }
    if (number < 5)
    { // less than 5, calculated directly statistics
        for (i = 0; i < number; i++)
        {
            amount += arr[i];
        }
        avg = amount / number;
        return avg;
    }
    else
    {
        if (arr[0] < arr[1])
        {
            min = arr[0];
            max = arr[1];
        }
        else
        {
            min = arr[1];
            max = arr[0];
        }
        for (i = 2; i < number; i++)
        {
            if (arr[i] < min)
            {
                amount += min; // arr<min
                min = arr[i];
            }
            else
            {
                if (arr[i] > max)
                {
                    amount += max; // arr>max
                    max = arr[i];
                }
                else
                {
                    amount += arr[i]; // min<=arr<=max
                }
            } // if
        }     // for
        avg = (double)amount / (number - 2);
    } // if
    return avg;
}