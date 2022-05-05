#include "stdio.h"                                                        // Library STDIO
#include "driver/ledc.h"                                                  // Library ESP32 LEDC
extern "C" {                                                              // bikin pusing ternyata harus di extern
  #include "soc/pcnt_struct.h"
}
#include "driver/pcnt.h"                                                  // Library ESP32 PCNT                                                      
#include <Wire.h>                                                                                                                                                                       
#include <LiquidCrystal.h>  
                                               
LiquidCrystal lcd(4, 16, 17, 5, 18, 19);                                  //   - ports RS,ENA,D4,D5,D6,D7
                                         
#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                               

#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // PIN34 input freq counter dikasih C104 gapapa engga juga nggapapa
#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // PIN33 Oscilator output
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // digabung PIN32 utk cek keluaran oscilator sendiri 
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // digabung PIN35 utk cek keluaran oscilator sendiri 
#define PCNT_H_LIM_VAL        overflow                                     

#define IN_BOARD_LED          GPIO_NUM_2                                  // Led indikator sinyal diukur
#define RELAY                 GPIO_NUM_23                                 // Relay
#define LCPilih               GPIO_NUM_36                                 // sensor pilih

bool            flag          = true;                                      
uint32_t        overflow      = 20000;                                     
int16_t         pulses        = 0;                                         
uint32_t        multPulses    = 0;                                         
uint32_t        sample_time   = 1000000;                                   
uint32_t        osc_freq      = 12543;                                    // Oscillator frequency awal  12543 Hz (bisa 1 Hz s.d. 40 MHz)
uint32_t        mDuty         = 0;                                         
uint32_t        resolusi      = 0;                                                                                 
char            buf[32]; 
                                                  
double frequency, capacitance, inductance;

esp_timer_create_args_t create_args;                                       
esp_timer_handle_t timer_handle;                                           
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 
                  
#define phi 3.14159
byte mode_operasi = 2;
double C_calb = 1.00;  //calibrasi  1.00 nF - adjust ini sangant menentukan seluruh hasil ukur
double L_calb;
bool dikalibrasi = true;

void setup(){
  Serial.begin(115200);                                                    
  Serial.println(" Input Frequency - 1 sd 40 MHz");                                                                     
  pinMode(RELAY, OUTPUT); digitalWrite(RELAY, LOW);  //boleh pasang boleh tidak
  pinMode(LCPilih, INPUT);  
                           
  lcd.begin(16, 2); 
  //lcd.print("  Frequency:");                                             
  init_frequencyMeter ();                                                 
}


void init_osc_freq ()  {
  resolusi = (log (80000000 / osc_freq)  / log(2)) / 2 ;                 
  if (resolusi < 1) resolusi = 1;                                      
  mDuty = (pow(2, resolusi)) / 2;                                        
  ledc_timer_config_t ledc_timer = {};                                     
  ledc_timer.duty_resolution =  ledc_timer_bit_t(resolusi);              
  ledc_timer.freq_hz    = osc_freq;                                        
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                            
  ledc_timer.timer_num = LEDC_TIMER_0;                                     
  ledc_timer_config(&ledc_timer);                                          
  ledc_channel_config_t ledc_channel = {};                                 
  ledc_channel.channel    = LEDC_CHANNEL_0;                                
  ledc_channel.duty       = mDuty;                                         
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                              
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                             
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                          
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                  
  ledc_channel_config(&ledc_channel);                                     
}

static void IRAM_ATTR pcnt_intr_handler(void *arg)  {
  portENTER_CRITICAL_ISR(&timerMux);                                       
  multPulses++; // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                 
  portEXIT_CRITICAL_ISR(&timerMux);                                       
}

void init_PCNT(void) {
  pcnt_config_t pcnt_config = { };                                         
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                          
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                          
  pcnt_config.unit = PCNT_COUNT_UNIT;                                      
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                                
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                              
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                   
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                   
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                              
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                 
  pcnt_unit_config(&pcnt_config);                                          
  pcnt_counter_pause(PCNT_COUNT_UNIT);                                     
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                     
  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                      
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                     
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                       
  pcnt_counter_resume(PCNT_COUNT_UNIT);                                    
}

void read_PCNT(void *p)  {
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                        
  flag = true;                                                             
}

void init_frequencyMeter (){
  init_osc_freq();                                                         
  init_PCNT();                                                             
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                               
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);               
  create_args.callback = read_PCNT;                                        
  esp_timer_create(&create_args, &timer_handle);                           
  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                      
  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);            
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);         
}


char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos) {
  int c;
  if (val >= radix) s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix; c += (c < 10 ? '0' : 'a' - 10); *s++ = c;
  if (pos % 3 == 0) *s++ = ',';
  return s;
}

char *ltos(long val, char *s, int radix){
  if (radix < 2 || radix > 36) { s[0] = 0; } else {
    char *p = s;
    if (radix == 10 && val < 0) { val = -val; *p++ = '-'; }
    p = ultos_recursive(val, p, radix, 0) - 1; *p = 0;
  }
  return s;
}

void baca_frequensi(){
  if (flag == true)  {
    flag = false;                                                        
    frequency = (pulses + (multPulses * overflow)) / 2  ;                
    //printf("Frequency : %s", (ltos(frequency, buf, 10)));               
    //printf(" Hz \n");                                                    

    lcd.setCursor(0, 0);  lcd.print("F:");lcd.print((ltos(frequency, buf, 10))); lcd.print(" Hz              ");                                      
 
    multPulses = 0;                                             
    delay (100);             
    pcnt_counter_clear(PCNT_COUNT_UNIT);                                
    esp_timer_start_once(timer_handle, sample_time);                    
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             
  }
}

void loop(){
    baca_frequensi();
    if (dikalibrasi==true)  kalibrasi();
    
      if (digitalRead(LCPilih)==LOW){
            capacitance = C_calb * 1e-9;
            inductance = (1. / (capacitance * frequency * frequency * 4.*phi * phi)) * 1.E6;  inductance = inductance - L_calb; 
            lcd.setCursor(0, 1);  lcd.print("L:");
            if(inductance>=1000) { lcd.print( inductance/1000.0 ); lcd.print(" mH            ");
            } else {lcd.print( inductance );lcd.print(" uH              ");}    
      }
      else if  (digitalRead(LCPilih)==HIGH){ 
            inductance = L_calb * 1e-6; 
            capacitance = ((1. / (inductance * frequency * frequency * 4.* phi * phi)) * 1.E9);
            capacitance=capacitance-C_calb;
            if((int)capacitance < 0) capacitance=0;
              lcd.setCursor(0, 1); lcd.print("C:");
            if(capacitance >= 100){ lcd.print(capacitance/1000.0);lcd.print(" uF                 "); }
            else if (capacitance >= 1 && capacitance <100){lcd.print(capacitance); lcd.print(" nF                 ");}
            else if (capacitance <1 ){
              if ((capacitance*1000)<0) {lcd.print(capacitance*0.0);} else {lcd.print(capacitance*1000.0);}
              lcd.print(" pF                 ");
              }
      }   
    ubah_oscilator_pakeSerial();
}

void ubah_oscilator_pakeSerial(){
  String inputString = "";                                               
  osc_freq = 0;                                                          
  while (Serial.available()) {
    char inChar = (char)Serial.read();  inputString += inChar;                                              
    if (inChar == '\n')  {
      osc_freq = inputString.toInt(); Serial.println(osc_freq);
      inputString = "";                                                  
    }
  }
  if (osc_freq != 0)  {init_osc_freq ();  }
}
 
void kalibrasi(){
 lcd.setCursor(0, 1); lcd.print("KALIBRASI..L");
            for (byte i = 1 ; i<10 ; i++){
                      baca_frequensi(); 
                      delay(500);
            }           
            C_calb = 1.0 * 1e-9; 
            L_calb = (1. / (C_calb * frequency * frequency * 4.*phi * phi)) * 1.E6;  
            lcd.setCursor(0, 1);  lcd.print("L:");
            if(L_calb>=1000) { lcd.print( L_calb/1000 ); lcd.print(" mH            ");}
            else {lcd.print( L_calb );lcd.print(" uH              ");}  
 delay(1500);
 lcd.setCursor(0, 1); lcd.print("KALIBRASI..C");
  
            for (byte i = 1 ; i<10 ; i++){
            baca_frequensi(); 
            delay(500);
             }          
            
            C_calb = ((1. / ((L_calb * 1e-6) * frequency * frequency * 4.* phi * phi)) * 1.E9);
            if((int)C_calb < 0) C_calb=0;
            lcd.setCursor(0, 1); lcd.print("C:");
            if(C_calb > 100){ lcd.print( (C_calb/1000));lcd.print(" uF                 "); }
            else {lcd.print(C_calb); lcd.print(" nF                 ");}
 
delay(2500);
dikalibrasi = false;
  
}
  
