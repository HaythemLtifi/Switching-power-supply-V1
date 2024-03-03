#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MCP4725.h> 
#include <SPI.h>
#include <SD.h>

#define ENC_DATA_PIN       3
#define ENC_CLK_PIN        2
#define ENC_SW_PIN         4
#define ENC_DATA_PIN_2       5
#define ENC_CLK_PIN_2        6
#define ENC_SW_PIN_2         7
#define MIN_Current           0
#define MAX_Current         3000
#define MAX_Current_2       6000
#define PRESS_TIME_SHORT   20 // [ms] short press
#define PRESS_TIME_LONG  800 // [ms] long  press
#define PRESS_TIME_VERY_LONG  2500 // [ms] long  press
#define ROTARY_FAST        10 // [ms] rotary speed fast
#define ROTARY_SLOW       200 // [ms] rotary speed slow
#define INCREMENT_SLOW      1
#define INCREMENT_FAST     10



byte increment;
byte enc_clk,enc_clk_2,enc_clk_old,enc_clk_old_2;
byte enc_switch,enc_switch_2, enc_switch_old,enc_switch_old_2;
float Aux_current_set,Main_current_set;
unsigned long  enc_pressed_time, enc_rotated_time;


LiquidCrystal_I2C lcd(0x27, 16, 2); 
int menuIndex = 0;

int maxMenuIndex = 2;
int maxSubMenuIndex = 1;

Adafruit_MCP4725 MCP4725,Main_DAC; 
Adafruit_INA219 ina219;
unsigned long MCP4725_Aux_current_set= 0;
unsigned long MCP4725_Main_current_set= 0;

  float busvoltage = 0;
  int current_mA = 0;
  float power_mW = 0;
  float Main_voltage;
  float Main_current_mA = 0;
  float MCP4725_AUX_value= 0;
  float MCP4725_MAIN_value= 0;
  float correction = 1.079;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 500;  //the value is a number of milliseconds
const unsigned long period2 = 1000;  //the value is a number of milliseconds

unsigned long previousMillis = 0;  // variable to store the previous time
unsigned long interval = 1000;     // default interval in milliseconds
boolean menuActive,Data_logging,file_create = false;  // flag to indicate if the menu is active

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 100;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
uint32_t total = 0;                  // the running total
int average = 0;                // the average

Sd2Card card;
File dataFile;



void setup() { 
    lcd.init();
    lcd.clear();         
    lcd.backlight(); 
  // iniate the table to hold the adc read values 
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
    Serial.begin (9600);
    
    //INA219 detecting
    if (!ina219.begin()) {
     Serial.println(F("Failed to find INA219 chip"));}
    //while (1) { delay(10); }}
    //SDcard detecting
   Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, 10)) {
    lcd.setCursor(0, 0);
    lcd.print(F("SD CARD ")); 
    lcd.setCursor(0, 1);
    lcd.print(F("NOT DETECTED")); 
    delay(2000);
    lcd.clear();
    } 
    
  else {
    SD.begin();
    Serial.println(F("SD card is present."));
    lcd.setCursor(0, 0);
    lcd.print(F("SD CARD DETECTED")); 
    delay(2000);
    lcd.clear();
  
  }
    ina219.setCalibration_32V_3A();
     // Make sure backlight is on
    pinMode (ENC_CLK_PIN,INPUT_PULLUP);
    pinMode (ENC_DATA_PIN,INPUT_PULLUP);
    pinMode (ENC_SW_PIN,INPUT_PULLUP);
    pinMode (ENC_CLK_PIN_2,INPUT_PULLUP);
    pinMode (ENC_DATA_PIN_2,INPUT_PULLUP);
    pinMode (ENC_SW_PIN_2,INPUT_PULLUP);
    MCP4725.begin(0x60);
    Main_DAC.begin(0x61);
    enc_clk_old    = digitalRead(ENC_CLK_PIN);
    enc_switch_old = digitalRead(ENC_SW_PIN);
    enc_clk_old_2    = digitalRead(ENC_CLK_PIN_2);
    enc_switch_old_2 = digitalRead(ENC_SW_PIN_2);

  updateLCD();
  startMillis = millis();  //initial start time
  analogReference(EXTERNAL);

lcd.setCursor(0, 0);
lcd.print(F("USB Serial ON")); 
lcd.setCursor(0, 1);
lcd.print(F("BAUD RATE: 9600"));  
delay(1500);
lcd.clear();

}

void loop() {
currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (menuActive) {
  
    } 
   if (Data_logging) {
  if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
     saveData();  // save data to SD card
      Serial.println(F("New data saved"));
   }
    }
    

//Print_Data();

update_Menu();
updateLCD();   
}


void Print_Data(){
  Serial.print(F("Bus Voltage:   ")); Serial.print(busvoltage); Serial.println(" V");
  Serial.print(F("Current:       ")); Serial.print(current_mA); Serial.println(" mA");
  Serial.print(F("Power:         ")); Serial.print(power_mW); Serial.println(" mW");
  Serial.println(F(""));
}
void update_Menu(){
enc_switch = digitalRead(ENC_SW_PIN);
  if(((enc_switch_old == 1) && enc_switch == 0)) enc_pressed_time = millis(); // 1->0 transition
  if((enc_switch_old == 0) && (enc_switch == 1)) { // 0->1 transition
    if((millis() - enc_pressed_time) > PRESS_TIME_VERY_LONG) { // very long press   
      Aux_current_set=0; 
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print(F("      Reset "));
      lcd.setCursor(0,1);
      lcd.print(F("   AUX_Current"));
      delay(1000);
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_LONG) { // long press
       // Button is long pressed, change to sub menu index

    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_SHORT) { // short press
       // Button is pressed, change the menu index
    menuIndex = (menuIndex + 1) % (maxMenuIndex + 1);
    if(menuIndex > 1) {   
     menuIndex=0;
                      }
    }    
  }
  enc_switch_old = enc_switch;
  enc_switch_2 = digitalRead(ENC_SW_PIN_2);
  if(((enc_switch_old_2 == 1) && enc_switch_2 == 0)) enc_pressed_time = millis(); // 1->0 transition

  if((enc_switch_old_2 == 0) && (enc_switch_2 == 1)) { // 0->1 transition
    if((millis() - enc_pressed_time) > PRESS_TIME_VERY_LONG) { // long press    
      Main_current_set=0; 
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print(F("      Reset "));
      lcd.setCursor(0,1);
      lcd.print(F("  MAIN_Current"));
      delay(1000);
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_LONG) { // long press
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_SHORT) { // short press
           // Button is pressed, change the menu index
    menuIndex = (menuIndex + 2) % (maxMenuIndex + 2);
    }   
  }
  enc_switch_old_2 = enc_switch_2;
  }



void updateCurrent(){
  // READ ROTARY ENCODER TO MODIFY AUX set current setting
  enc_clk = digitalRead(ENC_CLK_PIN);
  if((enc_clk == 1) && (enc_clk_old == 0)) { // 0->1 transition
    if((millis() - enc_rotated_time) > ROTARY_SLOW) {
      increment = INCREMENT_SLOW;  
    }
    else if((millis() - enc_rotated_time) > ROTARY_FAST) {
      increment = INCREMENT_FAST;  
    }
    else {
      //Serial.println("ROT BOUNCED");
      increment = 0;  
    }
    enc_rotated_time = millis();
    if(digitalRead(ENC_DATA_PIN) == 1) 
      Aux_current_set = Aux_current_set + increment;
    else
      Aux_current_set = Aux_current_set - increment;
    if(Aux_current_set < MIN_Current) {Aux_current_set = 3000;}
    if(Aux_current_set > MAX_Current) {Aux_current_set = 0;} 

  }
  enc_clk_old = enc_clk;
}
void updateMainCurrent(){
     // READ ROTARY ENCODER TO MODIFY Nain set current setting
  enc_clk_2 = digitalRead(ENC_CLK_PIN_2);
  if((enc_clk_2 == 1) && (enc_clk_old_2 == 0)) { // 0->1 transition
    if((millis() - enc_rotated_time) > ROTARY_SLOW) {
      increment = INCREMENT_SLOW;  
    }
    else if((millis() - enc_rotated_time) > ROTARY_FAST) {
      increment = INCREMENT_FAST;  
    }
    else {
      increment = 0;  
    }
    enc_rotated_time = millis();
    if(digitalRead(ENC_DATA_PIN_2) == 1){
      Main_current_set = Main_current_set + increment;}
    else{
      Main_current_set = Main_current_set - increment;
    }
    if(Main_current_set < MIN_Current) {Main_current_set = 6000;}
    if(Main_current_set > MAX_Current_2) {Main_current_set = 0;} 
  }
  enc_clk_old_2 = enc_clk_2;
}

void updateLCD() {
  lcd.setCursor(0, 0);
  if (menuIndex == 0) {
    if (currentMillis - startMillis >= period2)  //test whether the period has elapsed
  { update_INA219();
  Dac_OUTPUT();
  Dac_OUTPUT_MAIN();
  lcd.clear();
    // Add code here to display menu 1 contents
  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print(F("V1:"));
  analogReference(EXTERNAL);
  Main_voltage = smooth_ADC_read(A7);;
  Main_voltage = Main_voltage*33;
  Main_voltage = Main_voltage/1023;
  lcd.print(busvoltage,2);
  lcd.setCursor(8,0);   //Set cursor to character 2 on line 0
  lcd.print(F("V2:"));
  lcd.print(Main_voltage,2);
  lcd.setCursor(0,1);   //Move cursor to character 2 on line 1
  lcd.print(F("C1:"));
  analogReference(EXTERNAL);
  Main_current_mA = smooth_ADC_read(A6);
  Main_current_mA = Main_current_mA*3;
  Main_current_mA = Main_current_mA/1023;
  Main_current_mA = Main_current_mA / 0.4;
  Main_current_mA=Main_current_mA*1000;
  lcd.print(int(current_mA));
  lcd.setCursor(8,1);   
  lcd.print(F("C2:"));
  lcd.print((int)Main_current_mA);
    startMillis = currentMillis;  
  }
  } else if (menuIndex == 1) {
    updateCurrent();
      if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(F("Set AUX Current"));
    lcd.setCursor(0,1);  
    updateCurrent();
    lcd.print((int)Aux_current_set);
    lcd.print(F(" mA"));
    startMillis = currentMillis; 
  }
  }
    else if (menuIndex == 2) {
    updateMainCurrent();
      if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(F("Set MAIN Current"));
    lcd.setCursor(0,1);    
    lcd.print((int)Main_current_set);
    lcd.print(F(" mA"));
    startMillis = currentMillis; 
  }
  }
}

void Dac_OUTPUT(){
//Aux_current-=7;
double Aux_current = Aux_current_set/3000;
Aux_current = Aux_current*4095;
//Main_current-=7;
if (Aux_current<-1){Aux_current=0;}
MCP4725.setVoltage(Aux_current , false);
}
void Dac_OUTPUT_MAIN(){
double Main_current = Main_current_set/6000;
Main_current = Main_current*4095;
Main_current-=7;
if (Main_current<-1){Main_current=0;}
Main_DAC.setVoltage(Main_current , false);
}
void update_INA219(){
busvoltage = ina219.getBusVoltage_V();
current_mA = ina219.getCurrent_mA();
power_mW = ina219.getPower_mW();
}
float smooth_ADC_read(int inputPin){
for (int thisReading = 0; thisReading < numReadings; thisReading++) {
 analogReference(EXTERNAL);
 // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  }
  average = total / numReadings;
  return(average);     
}



void saveData() {
   dataFile = SD.open("data.txt", FILE_WRITE);
   if (dataFile) {
   if (!file_create) {createFile();}
   dataFile.println(F("Data entry"));  // example data entry
   dataFile.close();
    Serial.println(F("Data saved to SD card."));
   } else {
   Serial.println(F("Error opening data.txt"));
  }
}

void createFile() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F(" File data.txt"));
    lcd.setCursor(0, 1);
    lcd.print(F("  Was Created")); 
    delay(2000);
    lcd.clear();
    file_create = true;
}

void serialEvent() {
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'm') {
      // Enter menu mode
      menuActive = true;
      Serial.println(F("Menu Mode"));
      Serial.println(F("Commands:"));
      Serial.println(F("1 - Set interval : Default is 1000 ms"));
      Serial.println(F("2 - Start data logging"));
      Serial.println(F("3 - Stop data logging"));
      Serial.println(F("9 - Delete data.txt"));
      Serial.println();
      delay(100);
      }

    else if (input == '1') {
      // Enter interval value
      Serial.println(F("Enter the interval in milliseconds:"));
      while (!Serial.available()) {
        // Wait for user input
      }
      interval = Serial.parseInt();
      Serial.print(F("Interval set to "));
      Serial.print(interval);
      Serial.println(F(" ms"));
      Serial.println();
    } else if (input == '2') {
      // Start data logging
      menuActive = false;
      Data_logging = true;
      Serial.println(F("Data logging started."));
      Serial.println();
    } else if (input == '3') {
      // Start data logging
      menuActive = false;
      Data_logging = false;
      Serial.println(F("Data logging stoped."));
      dataFile = SD.open("data.txt", FILE_WRITE);
      dataFile.println(F("Data logging stoped."));  // example data entry
      dataFile.close();
      Serial.println();
    }
    else if (input == '9') {
      // Start data logging
      menuActive = false;
      Data_logging = false;
      Serial.println(F("Delete data.txt file."));
      SD.remove("data.txt");
      file_create = false;
      Serial.println();
    }
  
    } 
  }





