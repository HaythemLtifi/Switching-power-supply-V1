#define ENC_DATA_PIN       3
#define ENC_CLK_PIN        2
#define ENC_SW_PIN         4
#define MIN_Current           1
#define MAX_Current         3000
#define PRESS_TIME_SHORT   20 // [ms] short press
#define PRESS_TIME_LONG  1000 // [ms] long  press
#define PRESS_TIME_VERY_LONG  3000 // [ms] long  press
#define ROTARY_FAST        10 // [ms] rotary speed fast
#define ROTARY_SLOW       200 // [ms] rotary speed slow
#define INCREMENT_SLOW      1
#define INCREMENT_FAST     10


byte increment;
byte pointer; // pointer to setpoint[pointer]
byte enc_clk,    enc_clk_old;
byte enc_switch, enc_switch_old;

int Aux_current_set;

unsigned long  enc_pressed_time, enc_rotated_time;




void setup() { 
  pinMode (ENC_CLK_PIN,INPUT_PULLUP);
  pinMode (ENC_DATA_PIN,INPUT_PULLUP);
  pinMode (ENC_SW_PIN,INPUT_PULLUP);

  Serial.begin (9600);
  enc_clk_old    = digitalRead(ENC_CLK_PIN);
  enc_switch_old = digitalRead(ENC_SW_PIN);
}

void loop() {
// READ SWITCH AND CHANGE SERVO SETPOINT POINTER
  enc_switch = digitalRead(ENC_SW_PIN);
  if(((enc_switch_old == 1) && enc_switch == 0)) enc_pressed_time = millis(); // 1->0 transition

  if((enc_switch_old == 0) && (enc_switch == 1)) { // 0->1 transition
    if((millis() - enc_pressed_time) > PRESS_TIME_VERY_LONG) { // long press
      
      Serial.println("Reset Current");
      Aux_current_set=0;
   
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_LONG) { // long press
      
      Serial.println("Longs: press");
   
    }
    else if((millis() - enc_pressed_time) > PRESS_TIME_SHORT) { // short press
     
      Serial.println("Shorts: press ");

    }
    //else Serial.println("SW BOUNCED");
  }
  enc_switch_old = enc_switch;

// READ ROTARY ENCODER TO MODIFY SERVO SETPOINT
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
      //setpoint[pointer] = setpoint[pointer] + increment;
      Aux_current_set = Aux_current_set + increment;
    else
      Aux_current_set = Aux_current_set - increment;


      
    if(Aux_current_set < MIN_Current) Aux_current_set = 0;
    if(Aux_current_set > MAX_Current) Aux_current_set = 3000;
    Serial.println(Aux_current_set);
  }
  enc_clk_old = enc_clk;

  

}