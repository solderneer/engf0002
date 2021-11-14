/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
  This example code is in the public domain.
 */

#define YELLOW_BUTTON (11)
#define BLUE_BUTTON (10)
#define GREEN_BUTTON (9)
#define RED_BUTTON (8)
#define BUZZER (6) // Needs PWM

uint8_t a_buffer[4] = {0, 0, 0 ,0};
uint8_t buttons[] = {YELLOW_BUTTON, BLUE_BUTTON, GREEN_BUTTON, RED_BUTTON};
uint32_t button_tones[] = {262, 294, 330, 350};

/****** UTILITIES *********/

// Takes a 4 byte uint8_t array and converts it to a 32 bit unsigned integer
uint32_t bufferToUInt32(uint8_t *b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}


/********* MAIN CODE **************/
void setup() {                
  // Initialise the pushbuttons
	pinMode(buttons[0], INPUT_PULLUP);
  pinMode(buttons[1], INPUT_PULLUP);
  pinMode(buttons[2], INPUT_PULLUP);
  pinMode(buttons[3], INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  // Initialise the serial communication
  Serial.begin(9600);

  // Startup Sound
  tone(BUZZER, 330, 200);
  delay(200);
  noTone(BUZZER);
  tone(BUZZER, 262, 200);
  delay(200);
  noTone(BUZZER);
  tone(BUZZER, 330, 200);
  delay(200);
  noTone(BUZZER);
}

void loop() {
  // First check and play any input data
  if(Serial.available() > 0) {
    Serial.readBytesUntil('\n', a_buffer, 4);
    uint32_t freq = bufferToUInt32(a_buffer);
    Serial.print(freq);
    tone(BUZZER, freq, 1000); 
  }

  // Sample the buttons for tones
  int i;
  for(i = 0; i < 4; i++) {
    if(digitalRead(buttons[i]) == 0){
      tone(BUZZER, button_tones[i], 200);
      delay(100);
      noTone(BUZZER);
    }
  }
}
