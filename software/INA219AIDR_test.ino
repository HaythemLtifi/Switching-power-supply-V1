#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>

#include <Adafruit_MCP4725.h> // MCP4725 library from adafruit




LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display
// MCP4725 SETUP 
Adafruit_MCP4725 MCP4725; 

// INA219 SETUP 
Adafruit_INA219 ina219;
int MCP4725_value= 0;
float Data= 100;

void setup(void) 
{
  
  Serial.begin(115200);

    if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_3A();
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  MCP4725.begin(0x60);
    
  // To use a slightly lower 32V, 3A range (higher precision on amps):
  

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{  
   MCP4725_value= (Data * 4096) / 3000 ;
   
  
  
  lcd.clear();
  float busvoltage = 0;
  float current_mA = 0;
  float power_mW = 0;
  int dummy = 9999;


 
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("V1:");
  lcd.print(dummy,1);

  lcd.setCursor(9,0);   //Set cursor to character 2 on line 0
  lcd.print("V2:");
  lcd.print(busvoltage,1);

  
  lcd.setCursor(0,1);   //Move cursor to character 2 on line 1
  lcd.print("C1:");
  lcd.print(dummy);

  lcd.setCursor(9,1);   //Move cursor to character 2 on line 1
  lcd.print("C2:");
  lcd.print((int)current_mA);
  MCP4725.setVoltage(MCP4725_value, false);
  
  delay(1000);
}
