
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

    if (TU_value <= 0)
    {
        TU_value = 0;
    }
    if (TU_value >= 3000)
    {
        TU_value = 3000;
    }
    Serial.print("TU Value:");
    Serial.print(TU_value); // print out the value you read:
    Serial.println("NTU");
    // use json to transmit mqtt message
    // String output;
    float TDS = TU_value;
    doc["TDS"] = TDS;
    // serializeJson(doc, output);
    
    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish(topic, buffer, n);

    // String TUString = String(TU_value);
    // client.publish(topic,TUString);
    delay(1000);
    
}