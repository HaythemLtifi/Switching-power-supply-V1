#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MCP4725.h> 

#define ENC_DATA_PIN       3
#define ENC_CLK_PIN        2
#define ENC_SW_PIN         4
#define MIN_Current           0
#define MAX_Current         3000
#define PRESS_TIME_SHORT   20 // [ms] short press
#define PRESS_TIME_LONG  200 // [ms] long  press
#define PRESS_TIME_VERY_LONG  2000 // [ms] long  press
#define ROTARY_FAST        10 // [ms] rotary speed fast
#define ROTARY_SLOW       200 // [ms] rotary speed slow
#define INCREMENT_SLOW      1
#define INCREMENT_FAST     10

//#define DAC_RESOLUTION    (9);

byte increment;
byte enc_clk,    enc_clk_old;
byte enc_switch, enc_switch_old;
float Aux_current_set;
unsigned long  enc_pressed_time, enc_rotated_time;


LiquidCrystal_I2C lcd(0x27, 16, 2);
int menuIndex = 0;
int maxMenuIndex = 1;

Adafruit_MCP4725 MCP4725; 
Adafruit_INA219 ina219;
unsigned long MCP4725_Aux_current_set= 0;

  float busvoltage = 0;
  int current_mA = 0;
  float power_mW = 0;
  int dummy = 9999;
  float MCP4725_AUX_value= 0;
  float correction = 1.079;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 500;  //the value is a number of milliseconds
const unsigned long period2 = 1000;  //the value is a number of milliseconds

void setup() { 
  
    Serial.begin (9600);
    if (! ina219.begin()) {
     Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }}
    ina219.setCalibration_32V_3A();
    lcd.init();
    lcd.clear();         
    lcd.backlight();      // Make sure backlight is on
    lcd.setCursor(0, 0);
    lcd.print("Menu 1");
    
    pinMode (ENC_CLK_PIN,INPUT_PULLUP);
    pinMode (ENC_DATA_PIN,INPUT_PULLUP);
    pinMode (ENC_SW_PIN,INPUT_PULLUP);
    MCP4725.begin(0x60);
 
    enc_clk_old    = digitalRead(ENC_CLK_PIN);
    enc_switch_old = digitalRead(ENC_SW_PIN);
    lcd.clear();
  updateLCD();
  startMillis = millis();  //initial start time
}

void loop() {

//Print_Data();
currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
update_Menu();
updateLCD();  

   
}


void Print_Data(){
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}



void update_Menu(){enc_switch = digitalRead(ENC_SW_PIN);
  if(((enc_switch_old == 1) && enc_switch == 0)) enc_pressed_time = millis(); // 1->0 transition

  if((enc_switch_old == 0) && (enc_switch == 1)) { // 0->1 transition
    if((millis() - enc_pressed_time) > PRESS_TIME_VERY_LONG) { // long press
      
      Serial.println("Reset Current");
      Aux_current_set=0;
      Serial.println(Aux_current_set);
   
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_LONG) { // long press
      
      Serial.println("Longs: press");
   
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_SHORT) { // short press
     
      Serial.println("Shorts: press ");
       // Button is pressed, change the menu index
    menuIndex = (menuIndex + 1) % (maxMenuIndex + 1);
    }
    
  }
  enc_switch_old = enc_switch;}


void updateCurrent(){
 

  // READ ROTARY ENCODER TO MODIFY Scurrent setting
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


      
    if(Aux_current_set <= MIN_Current) {Aux_current_set = 0;}
    if(Aux_current_set >= MAX_Current) {Aux_current_set = 3000;} 
    Serial.println(Aux_current_set);
  }
  enc_clk_old = enc_clk;
}

void updateLCD() {
  
  lcd.setCursor(0, 0);
  
  if (menuIndex == 0) {
 
    if (currentMillis - startMillis >= period2)  //test whether the period has elapsed
  { update_INA219();
  Dac_OUTPUT();
  lcd.clear();
    // Add code here to display menu 1 contents
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
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
  
    


  } else if (menuIndex == 1) {
    updateCurrent();
      if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(" Set AUX Current");
    lcd.setCursor(0,1);  
    updateCurrent();
    lcd.print((int)Aux_current_set);
    lcd.print(" mA");
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
   
  }
}
void Dac_OUTPUT(){
 
 Serial.print("Aux_current_set:");
 Serial.println(Aux_current_set);
int Aux_current = map(Aux_current_set, 0, 3000 , 0, 4096 );
Aux_current-=7;
if (Aux_current<0){Aux_current=0;}
MCP4725.setVoltage(Aux_current , false);
}


void update_INA219(){
busvoltage = ina219.getBusVoltage_V();
current_mA = ina219.getCurrent_mA();
power_mW = ina219.getPower_mW();
}
