#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // Use UART2 for GPS (RX=16, TX=17)
QMC5883LCompass compass;

// Initialize U8g2 for SSD1106 (128x64 I2C Display)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
    Serial.begin(115200);

    Wire.begin();
    Serial.println("\nI2C Scanner");
    for (uint8_t address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }

    // Initialize GPS
    gpsSerial.begin(115200, SERIAL_8N1, 16, 17);  // GPS on UART2 (RX=16, TX=17)

    // Initialize Display
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);

    // Initialize Compass
    compass.init();
    compass.setMode(0x01, 0x0C, 0x10, 0x00); // Continuous, 200Hz, 8G, 512 (High Power)

    delay(100);
}

String getCardinalDirection(float headingDegrees) {
  // Array of directions
  const char* directions[] = {
    " N", " NNE", " NE", " ENE", " E", " ESE", " SE", " SSE", " S", " SSW", 
    " SW", " WSW", " W", " WNW", " NW", " NNW"
  };

  // Array of ranges for each direction
  float ranges[] = { 11.25, 33.75, 56.25, 78.75, 101.25, 123.75, 146.25, 168.75, 
                     191.25, 213.75, 236.25, 258.75, 281.25, 303.75, 326.25, 348.75 };

  // Determine the index of the cardinal direction
  int index = 0;

  // Special case for North (0-11.25 and 348.75-360)
  if (headingDegrees < 11.25 || headingDegrees > 348.75) {
    return directions[0]; // North
  }

  // Find the appropriate direction based on the headingDegrees
  for (int i = 0; i < 16; i++) {
    if (headingDegrees <= ranges[i]) {
      index = i;
      break;
    }
  }

  return directions[index];
}

void loop() {
    // Read compass
    int x, y, z;
    String cardinal;
    
    compass.read();

    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    float headingRadians = atan2(y, x);
    float headingDegrees = headingRadians * 180 / PI;
    float declinationAngle = -12.566666667; // NYC
  
    headingDegrees += declinationAngle;
  
    headingDegrees = fmod(headingDegrees + 360.0, 360.0); 

    cardinal = getCardinalDirection(headingDegrees);

    // Read GPS
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Display data on SSD1106 using U8g2
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_6x12_tf);
    
    // Heading Display
    u8g2.setCursor(10, 15);
    u8g2.print("Heading: ");
    u8g2.setCursor(60, 15);
    u8g2.print(headingDegrees);
    //u8g2.print("°");

    // GPS Time Display
    u8g2.setCursor(10, 25);
    if (gps.time.isValid()) {
        u8g2.printf("Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        u8g2.print("No GPS Fix");
    }

    // GPS Location Display
    u8g2.setCursor(10, 50);
    if (gps.location.isUpdated()) {
        u8g2.printf("Lat: %.6f", gps.location.lat());
        u8g2.setCursor(10, 60);
        u8g2.printf("Lon: %.6f", gps.location.lng());
    }

    // GPS Satellites Display
    u8g2.setCursor(10, 35);
    u8g2.print("Sat: ");
    u8g2.print(gps.satellites.value());

    u8g2.sendBuffer();  // Send data to the display

    Serial.printf("Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.printf("GPS - Lat: %.6f, Lon: %.6f, Sats: %d\n", gps.location.lat(), gps.location.lng(), gps.satellites.value());
    Serial.printf("Compass - Heading: %.1f°, Dir: %s, X: %d, Y: %d, Z: %d, Mag: %.1f\n\n", headingDegrees, cardinal, x, y, z, sqrt(x * x + y * y + z * z));

    delay(500);
}

