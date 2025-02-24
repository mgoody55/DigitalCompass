#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <ezButton.h>
#include <RotaryEncoder.h>

// Hardware Components
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
QMC5883LCompass compass;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Constants
const int updateInterval = 500;
unsigned long lastUpdate = 0;

// Encoder Configuration
#define ENCODER_CLK 25
#define ENCODER_DT 26
#define ENCODER_SW 27
ezButton button(ENCODER_SW);

int page = 0;
int max_page = 3;

void scanI2CDevices() {
    Serial.println("\nI2C Scanner");
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
}

void initGPS() {
    gpsSerial.begin(115200, SERIAL_8N1, 16, 17);  // GPS on UART2 (RX=16, TX=17)
}

void initCompass() {
    compass.init();
    compass.setMode(0x01, 0x0C, 0x10, 0x00); // Continuous, 200Hz, 8G, 512 (High Power)
}

void initDisplay() {
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    button.setDebounceTime(25);  // Configure Encoder button debounce
    
    scanI2CDevices();
    initGPS();
    initCompass();
    initDisplay();

    delay(100);
}


String getCardinalDirection(float headingDegrees) {
    const char* directions[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
    float ranges[] = { 11.25, 33.75, 56.25, 78.75, 101.25, 123.75, 146.25, 168.75,
                      191.25, 213.75, 236.25, 258.75, 281.25, 303.75, 326.25, 348.75 };
    if (headingDegrees < 11.25 || headingDegrees > 348.75) return "N";
    for (int i = 0; i < 16; i++) {
        if (headingDegrees <= ranges[i]) return directions[i];
    }
    return "N"; 
}

void handleButtonPress() {
    static unsigned long pressStartTime = 0;
    button.loop();
    
    if (button.isPressed()) {
        pressStartTime = millis();
    }

    if (button.isReleased()) {
        unsigned long pressDuration = millis() - pressStartTime;
        if (pressDuration >= 1000) {
            Serial.println("Long press detected!");
        } else {
            page = (page + 1) % (max_page + 1);
            Serial.println(page);
        }
    }
}

void readCompass(float &headingDegrees, String &cardinal) {
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    headingDegrees = atan2(y, x) * 180 / PI;
    headingDegrees += -12.57;  // NYC declination
    headingDegrees = fmod(headingDegrees + 360.0, 360.0);
    cardinal = getCardinalDirection(headingDegrees);
    Serial.printf("Heading: %.1f, Dir: %s, Mag: %.1f\n", headingDegrees, cardinal, sqrt(x * x + y * y + z * z));
}

void readGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
      }
    Serial.printf("Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.printf("GPS - Lat: %.6f, Lon: %.6f, Sats: %d\n", gps.location.lat(), gps.location.lng(), gps.satellites.value());
}

void updateDisplay(float headingDegrees, String cardinal) {
    u8g2.clearBuffer();
    u8g2.setCursor(10, 15);
    u8g2.print("Heading: ");
    u8g2.setCursor(60, 15);
    u8g2.print(headingDegrees);

    u8g2.setCursor(10, 25);
    if (gps.time.isValid()) {
        u8g2.printf("Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        u8g2.print("No GPS Fix");
    }

    u8g2.setCursor(10, 50);
    if (gps.location.isUpdated()) {
        u8g2.printf("Lat: %.6f", gps.location.lat());
        u8g2.setCursor(10, 60);
        u8g2.printf("Lon: %.6f", gps.location.lng());
    }

    u8g2.setCursor(10, 35);
    u8g2.print("Sat: ");
    u8g2.print(gps.satellites.value());

    u8g2.sendBuffer();
}

void loop() {
    handleButtonPress();

    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();
        
        float headingDegrees;
        String cardinal;
        readCompass(headingDegrees, cardinal);
        readGPS();
        updateDisplay(headingDegrees, cardinal);
        
        Serial.println();
    }

}
