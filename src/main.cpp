#include <Arduino.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// Pin definitions
#define ONE_WIRE_BUS PA4        // Pin for DS18B20 sensors
#define DHT_PIN PA1             // Pin for DHT11 sensor
#define HEATER_RELAY PB8    // Pin for heater relay
#define FAN_RELAY PB9       // Pin for fan relay
#define SERVO PB10           // Pin for servo motor
#define BUTTON_TEMP_UP PB12          // Pin for button (for adjusting temperature value, positive)
#define BUTTON_TEMP_DOWN PA7          // Pin for button (for adjusting temperature value, negative)
#define BUTTON_HUM_UP PA8          // Pin for button (for adjusting humiidity value, positive)
#define BUTTON_HUM_DOWN PA11          // Pin for button (for adjusting humiidity value, negative)

// Define pins for the 16x4 LCD (RS, E, D4, D5, D6, D7)
#define LCD_RS PB5
#define LCD_Enable PB4
#define LCD_D4 PB0
#define LCD_D5 PB1
#define LCD_D6 PB3
#define LCD_D7 PB6

#define DHTTYPE DHT11          // DHT11 sensor type

// Initialize components
OneWire oneWire(ONE_WIRE_BUS);
DHT dht(DHT_PIN, DHTTYPE);
DallasTemperature ds18b20(&oneWire);
DeviceAddress tempSensor1, tempSensor2;
LiquidCrystal lcd(LCD_RS, LCD_Enable, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 

// Variables
float topTemp, bottomTemp, middleTemp;
float humidity;
int eggTiltAngle = 0;
float setTemperature = 37.5;   // Default set temperature
float setHumidity = 60.0;      // Default set humidity

// Function to initialize components
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
    Serial.println("Egg incubator");

  // Initialize sensors
  dht.begin();
  ds18b20.begin();
  
  
  // Assign addresses manually to each sensor
  if (ds18b20.getAddress(tempSensor1, 0)) {
    Serial.println("Sensor 1 found.");
    }
  if (ds18b20.getAddress(tempSensor2, 1)) {
    Serial.println("Sensor 2 found.");
    }

  // Initialize LCD
  lcd.begin(16, 4);  // Set LCD size to 16x4

  // Initialize relays and servo
  pinMode(HEATER_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(SERVO, OUTPUT);


  pinMode(BUTTON_TEMP_UP, INPUT_PULLDOWN );
  pinMode(BUTTON_TEMP_DOWN, INPUT_PULLDOWN );
  pinMode(BUTTON_HUM_UP, INPUT_PULLDOWN );
  pinMode(BUTTON_HUM_DOWN, INPUT_PULLDOWN );

  

  // Set initial relay and servo states
  digitalWrite(HEATER_RELAY, LOW);
  digitalWrite(FAN_RELAY, LOW);



  // Welcome message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Egg Incubator");
  delay(1000);
  lcd.clear();
}

// Function to move the servo to a specific angle (0° to 180°)
void moveServo(int angle) {
  // Calculate the pulse width in microseconds
  int pulseWidth = map(angle, 0, 180, 1000, 2000); // 1000us = 1ms, 2000us = 2ms

  // Send the pulse to the servo
  unsigned long startTime = millis();  // Record the start time

while (millis() - startTime < 500) {
  digitalWrite(SERVO, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO, LOW);
  delay(20);  // 20ms corresponds to the 50Hz servo refresh rate
}

}

// Function to read sensor data
void readSensors() {
  ds18b20.requestTemperatures();
  topTemp = ds18b20.getTempC(tempSensor1);      // Top sensor
  bottomTemp = ds18b20.getTempC(tempSensor2);   // Bottom sensor
  middleTemp = dht.readTemperature();        // Middle sensor (DHT11)
  humidity = dht.readHumidity();             // Humidity from DHT11
}

// Function to control heating and fan
void controlHeating() {
  float avgTemp = (topTemp + bottomTemp + middleTemp) / 3;

  if (avgTemp < setTemperature) {
    digitalWrite(HEATER_RELAY, HIGH);  // Turn on heater
    digitalWrite(FAN_RELAY, HIGH);     // Turn on fan with heater
  } else {
    digitalWrite(HEATER_RELAY, LOW);   // Turn off heater
    digitalWrite(FAN_RELAY, LOW);      // Turn off fan
  }
}

// Function to control egg tilting
void controlEggTilting() {
  // Tilt eggs every 4 hours 
  if (millis() % (4 * 60 * 60 * 1000) < 1000) {
    eggTiltAngle = (eggTiltAngle == 0) ? 45 : 0;  // Alternate between 0 and 45 degrees
    moveServo(eggTiltAngle);
  }
}

// Function to update the LCD to display current and set temperature
void updateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("T:");   // current temperature
  lcd.setCursor( 3 , 0);
  lcd.print((topTemp + bottomTemp + middleTemp) / 3, 1);
  lcd.print("C");

  lcd.setCursor(8, 0);
  lcd.print("H:");  // current Humidity
  lcd.setCursor(11, 0);
  lcd.print(humidity, 1);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("PT:"); // set temperature 
  lcd.setCursor(4, 1);
  lcd.print(setTemperature, 1);

  lcd.setCursor(8, 1);
  lcd.print("SH:");  // set Humidity
  lcd.setCursor(12, 1);
  lcd.print(setHumidity, 1);
}

// Function to handle button press for adjustments
void handleButtonPress() {
  if (digitalRead(BUTTON_TEMP_UP) == HIGH) {
    Serial.println("Temperature Incrementing");
    setTemperature +=0.5;
    delay(300);                // Debounce button press
  }
  if (digitalRead(BUTTON_TEMP_DOWN) == HIGH) {
    Serial.println("Temperature decrementing");
    setTemperature -=0.5;
    delay(300);                // Debounce button press
  }
  if (digitalRead(BUTTON_HUM_UP) == HIGH) {
    Serial.println("Humidity Incrementing");
    setHumidity +=0.5;
    delay(300);                // Debounce button press
  }
  if (digitalRead(BUTTON_HUM_DOWN) == HIGH) {
    Serial.println("Humidity decrementing");
    setHumidity -=0.5;
    delay(300);                // Debounce button press
  }

}

void loop() {
  
  // Read sensors
  readSensors();

  // Control heating and fan
  controlHeating();

  // Control egg tilting
  controlEggTilting();

  // Update display
  updateDisplay();

  // Handle button press
  handleButtonPress();

  // delay(1000);  // Wait 1 second between loops
}
