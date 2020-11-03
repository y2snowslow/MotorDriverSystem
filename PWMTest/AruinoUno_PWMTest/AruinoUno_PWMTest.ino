/* Sensorless brushless DC (BLDC) motor control with Arduino UNO (Arduino DIY ESC).
 * This is a free software with NO WARRANTY.
 * https://simple-circuit.com/
 */
 
 
#define SPEED_UP          A0          // BLDC motor speed-up button
#define SPEED_DOWN        A1          // BLDC motor speed-down button
#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      50
#define PWM_START_DUTY    100

const uint8_t sinewaveLUT[] PROGMEM = {
/*     0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x98, 0x9c, 0x9f, 0xa2,
    0xa5, 0xa8, 0xab, 0xae, 0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbf, 0xc1, 0xc4,
    0xc7, 0xc9, 0xcc, 0xce, 0xd1, 0xd3, 0xd5, 0xd8, 0xda, 0xdc, 0xde, 0xe0,
    0xe2, 0xe4, 0xe6, 0xe8, 0xea, 0xec, 0xed, 0xef, 0xf0, 0xf2, 0xf3, 0xf5,
    0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xfe, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe,
    0xfd, 0xfc, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6, 0xf5, 0xf3, 0xf2,
    0xf0, 0xef, 0xed, 0xec, 0xea, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc,
    0xda, 0xd8, 0xd5, 0xd3, 0xd1, 0xce, 0xcc, 0xc9, 0xc7, 0xc4, 0xc1, 0xbf,
    0xbc, 0xb9, 0xb6, 0xb3, 0xb0, 0xae, 0xab, 0xa8, 0xa5, 0xa2, 0x9f, 0x9c,
    0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83, 0x80, 0x7c, 0x79, 0x76,
    0x73, 0x70, 0x6d, 0x6a, 0x67, 0x63, 0x60, 0x5d, 0x5a, 0x57, 0x54, 0x51,
    0x4f, 0x4c, 0x49, 0x46, 0x43, 0x40, 0x3e, 0x3b, 0x38, 0x36, 0x33, 0x31,
    0x2e, 0x2c, 0x2a, 0x27, 0x25, 0x23, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x17,
    0x15, 0x13, 0x12, 0x10, 0x0f, 0x0d, 0x0c, 0x0a, 0x09, 0x08, 0x07, 0x06,
    0x05, 0x04, 0x03, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0c, 0x0d, 0x0f, 0x10, 0x12, 0x13,
    0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x23, 0x25, 0x27, 0x2a, 0x2c,
    0x2e, 0x31, 0x33, 0x36, 0x38, 0x3b, 0x3e, 0x40, 0x43, 0x46, 0x49, 0x4c,
    0x4f, 0x51, 0x54, 0x57, 0x5a, 0x5d, 0x60, 0x63, 0x67, 0x6a, 0x6d, 0x70,
    0x73, 0x76, 0x79, 0x7c*/
    128,131,134,137,140,144,147,150,153,156,159,162,
    165,168,171,174,177,180,182,185,188,191,194,196,
    199,201,204,206,209,211,214,216,218,220,222,224,
    226,228,230,232,234,236,237,239,240,242,243,244,
    246,247,248,249,250,251,251,252,253,253,254,254,
    254,255,255,255,255,255,255,255,254,254,253,253,
    252,252,251,250,249,248,247,246,245,244,242,241,
    240,238,236,235,233,231,229,227,225,223,221,219,
    217,215,212,210,208,205,203,200,197,195,192,189,
    187,184,181,178,175,172,169,167,164,160,157,154,
    151,148,145,142,139,136,133,130,126,123,120,117,
    114,111,108,105,102,99,96,92,89,87,84,81,
    78,75,72,69,67,64,61,59,56,53,51,48,
    46,44,41,39,37,35,33,31,29,27,25,23,
    21,20,18,16,15,14,12,11,10,9,8,7,
    6,5,4,4,3,3,2,2,1,1,1,1,
    1,1,1,2,2,2,3,3,4,5,5,6,
    7,8,9,10,12,13,14,16,17,19,20,22,
    24,26,28,30,32,34,36,38,40,42,45,47,
    50,52,55,57,60,62,65,68,71,74,76,79,
    82,85,88,91,94,97,100,103,106,109,112,116,
    119,122,125,128
    };

 
byte bldc_step = 0, motor_speed, motor_pos = 0;

unsigned int i;
void setup() {
  DDRD  |= 0x38;           // Configure pins 3, 4 and 5 as outputs
  PORTD  = 0x00;

  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);

  DDRB  |= 0x0E;           // Configure pins 9, 10 and 11 as outputs
  PORTB  = 0x31;
  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A = 0;
  TCCR2B = 0x01;
  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
  pinMode(SPEED_UP,   INPUT_PULLUP);
  pinMode(SPEED_DOWN, INPUT_PULLUP);
}
// Analog comparator ISR
/*ISR (ANALOG_COMP_vect) {
  // BEMF debounce
  for(i = 0; i < 10; i++) {
    if(bldc_step & 1){
      if(!(ACSR & 0x20)) i -= 1;
    }
    else {
      if((ACSR & 0x20))  i -= 1;
    }
  }
  bldc_move();
  bldc_step++;
  bldc_step %= 6;
}
*/
/*
void bldc_move(){        // BLDC motor commutation function
  switch(bldc_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}
*/

void loop() {

  // Enter Enable pin9,10,11 PWM 
  TCCR1A =  0xA1;         //
  TCCR2A =  0x81;         //
  while(1){
  //
  SET_PWM_DUTY(motor_pos);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  delay(1); // 1ms delay
  motor_pos += 1;

  }
  /*i = 5000;
  // Motor start
  while(i > 100) {
    delayMicroseconds(i);
    bldc_move();
    bldc_step++;
    bldc_step %= 6;
    i = i - 20;
  }*/
  //motor_speed = PWM_START_DUTY;
  //ACSR |= 0x08;                    // Enable analog comparator interrupt
  
  //Veryfy Output
  //TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  //TCCR1A =  0x81;         //
  //TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  //TCCR1A =  0x21;         //
  //TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  //TCCR2A =  0x81;         //
 /*
  while(1) {
    while(!(digitalRead(SPEED_UP)) && motor_speed < PWM_MAX_DUTY){
      motor_speed++;
      SET_PWM_DUTY(motor_speed);
      delay(1);
    }
    while(!(digitalRead(SPEED_DOWN)) && motor_speed > PWM_MIN_DUTY){
      motor_speed--;
      SET_PWM_DUTY(motor_speed);
      delay(1);
    }
  }
  */
}

void SET_PWM_DUTY(byte duty){
/*  if(duty < PWM_MIN_DUTY)
    duty  = PWM_MIN_DUTY;
  if(duty > PWM_MAX_DUTY)
    duty  = PWM_MAX_DUTY;
    */
  unsigned int j = (int)duty;
  OCR1A  = (byte)(j);                   // Set pin 9  PWM duty cycle
  OCR1B  = (byte)( (j + 85) % 256);     // Set pin 10 PWM duty cycle
  OCR2A  = (byte)( (j + 170) % 256);     // Set pin 11 PWM duty cycle
}


/*
void BEMF_A_RISING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR |= 0x03;            // Set interrupt on rising edge
}
void BEMF_A_FALLING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR &= ~0x01;           // Set interrupt on falling edge
}
void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= 0x03;
}
void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01;
}
void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= 0x03;
}
void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR &= ~0x01;
}
 
void AH_BL(){
  //PORTD &= ~0x28;
  //PORTD |=  0x10;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void AH_CL(){
  //PORTD &= ~0x30;
  //PORTD |=  0x08;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void BH_CL(){
  //PORTD &= ~0x30;
  //PORTD |=  0x08;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void BH_AL(){
  //PORTD &= ~0x18;
  //PORTD |=  0x20;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void CH_AL(){
  //PORTD &= ~0x18;
  //PORTD |=  0x20;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
}
void CH_BL(){
  //PORTD &= ~0x28;
  //PORTD |=  0x10;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
}
 
*/
