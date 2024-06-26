
// wifi mqtt json

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>

// 设备id
#define dev_id 1

// WiFi
const char *ssid = "scxazy";       // Enter your WiFi name
const char *password = "cx020629"; // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "114.55.251.57";
const char *pub_topic = "scx/pub";
const char *sub_topic = "scx/sub";
const char *mqtt_username = "scx";
const char *mqtt_password = "test";
const int mqtt_port = 1883;

// mqtt json 
StaticJsonDocument<200> doc;
// 存入MySQL的 flag
// flag 为 1 后 再存入数据
int sqlFlag = 0;

// TDS 所需要的 参数
float TU = 0.0;
float TU_value = 0.0;
float TU_calibration = 0.0;
float temp_data = 25.0;
// 这个温度应该是要被读取的 但是默认现在是室温环境好了
// 2024 4 15 温度传感器 
float K_Value = 3347.19;
#define SensorPinTDS 39

// PH 所需要的参数
#define SensorPinPH 36            //pH meter Analog output to Arduino Analog Input 2


#define Offset 12.88           //deviation compensate
// #define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;  

#define SensorPinTemp 34

// temp 配置
#include <OneWireNg.h>  // 使用ng的 不要使用
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

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
    client.publish(pub_topic, "Hi EMQ X I'm ESP32 ^^");
    
    // 水质检测器检测 更改 sqlFlag
    client.subscribe(sub_topic);
    sensors.begin();
}

// 回调代码
void callback(char *pub_topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived in pub_topic: ");
    Serial.println(pub_topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
    sqlFlag = 1;
}

void loop()
{
    client.loop();
    // temp 获取
    sensors.requestTemperatures(); // 发送命令获取温度
    temp_data = sensors.getTempCByIndex(0);
    
    Serial.print("Temp: ");
    Serial.println(temp_data);
    
    
    
    int sensorValue = analogRead(SensorPinTDS);        // read the input on analog pin 0:
    float TU = sensorValue * (3.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    TU_calibration = -0.0192 * (temp_data - 25) + TU;
    TU_value = -865.68 * TU_calibration + K_Value;
    int testTDSABS = sensorValue-2100;
    testTDSABS = abs(testTDSABS);
    if(testTDSABS<=13){
      testTDSABS = 0;
    }
    Serial.print("testTDSABS ");
    Serial.println(testTDSABS);
    
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
    // float TDS = TU_value;
    float TDS = TU_value/3;
    float PH = 7.0;



    // ph 数据 获取
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue, voltage;
    if (millis() - samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++] = analogRead(SensorPinPH);
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

    float TEMP = temp_data;
    float tempPH = PH - 7;
    float absPH = abs(tempPH);
    Serial.print("abs: ");
    Serial.println(absPH);
    float SCORE = (1000-TDS)/1000*50+(7-absPH)/7*50;
    Serial.print("SCORE: ");
    Serial.println(SCORE);
    // json 数据 发送 mqtt
    doc["dev_id"] = dev_id;
    doc["dev_name"] = "dev_1";
    doc["SCORE"] = SCORE;
    doc["TDS"] = TDS;
    doc["PH"] = PH;
    doc["TEMP"] = TEMP;
    doc["sqlFlag"] = sqlFlag;
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
    client.publish(pub_topic, buffer, n);
    
    
    if(sqlFlag == 1){
        sqlFlag = 0;
    }


    // String TUString = String(TU_value);
    // client.publish(pub_topic,TUString);
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