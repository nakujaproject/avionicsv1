#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>

const char *AP_SSID = "Mwesi";
const char *AP_PWD = "Tembea101";

WiFiMulti wifiMulti;

void setup()
{
    Serial.begin(115200);

    delay(4000);
    wifiMulti.addAP(AP_SSID, AP_PWD);
    postDataToServer();
}

void loop()
{
    getDataFromServer();
    delay(2000);
}

void postDataToServer()
{

    Serial.println("Posting JSON data to server...");
    // Block until we are able to connect to the WiFi access point
    if (wifiMulti.run() == WL_CONNECTED)
    {

        HTTPClient http;
        http.begin("http://98df33f8e2bb.ngrok.io/avionics/init-done");
        http.addHeader("Content-Type", "application/json");

        StaticJsonDocument<200> doc;
        doc["status"] = "Done";

        String requestBody;
        serializeJson(doc, requestBody);

        int httpResponseCode = http.POST(requestBody);

        if (httpResponseCode > 0)
        {
            String response = http.getString();
            Serial.println(response);
        }
        else
        {
            Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
        }
    }
}

void getDataFromServer()
{
    Serial.println("Getting JSON data from server...");
    // Block until we are able to connect to the WiFi access point
    if (wifiMulti.run() == WL_CONNECTED)
    {
        HTTPClient http;
        http.begin("http://98df33f8e2bb.ngrok.io/avionics/start-logging");
        http.GET();

        DynamicJsonDocument doc(2048);
        deserializeJson(doc, http.getStream());

        Serial.println(doc["status"].as<String>());

        http.end();
    }
}