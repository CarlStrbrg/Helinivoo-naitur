#include <LiquidCrystal.h>
#include <Arduino.h>
// Initialize the LiquidCrystal library with the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int N_LP = 10;
const int N_HP = 10;
const int N_BP1 = 16;
const int N_BP2 = 16;

float b_LP[N_LP + 1] = {
    0.000000000000000001,
    0.009304283145018143,
    0.047577766134417436,
    0.122363546361144632,
    0.202246558429840267,
    0.237015691859158939,
    0.202246558429840323,
    0.122363546361144687,
    0.047577766134417450,
    0.009304283145018147,
    0.000000000000000001,
};

float b_HP[N_HP + 1] = {
    0.000000000000000002,
    0.007855854095023677,
    -0.040171175263434403,
    0.103314801557551267,
    -0.170762156469434240,
    0.199525352160587310,
    -0.170762156469434268,
    0.103314801557551308,
    -0.040171175263434410,
    0.007855854095023681,
    0.000000000000000002,
};

float b_BP1[N_BP1+1] = {
    -0.000021917777270288,
    -0.000030169979189505,
    0.000240604443173214,
    0.000725838448390775,
    0.000117411380764189,
    -0.055367874045989487,
    -0.147281869494109818,
    0.053975661406001629,
    0.295284631236466644,
    0.053975661406002254,
    -0.147281869494110762,
    -0.055367874045989571,
    0.000117411380764172,
    0.000725838448390749,
    0.000240604443173212,
    -0.000030169979189486,
    -0.000021917777270200,
};

float b_BP2[N_BP2+1] = {
    -0.000005834754777186,
    0.000126791233054575,
    0.000190676595302891,
    0.000488764387682420,
    -0.004119801511072464,
    0.006255113514775717,
    -0.166905837686144642,
    -0.116735617604701025,
    0.561411491651768513,
    -0.116735617604701955,
    -0.166905837686145170,
    0.006255113514775750,
    -0.004119801511072471,
    0.000488764387682439,
    0.000190676595302902,
    0.000126791233054548,
    -0.000005834754777042,
};

int inputBuffer[N_BP1 + 1] = {0};
//int inputBuffer_HP[N_HP + 1] = {0};
//int inputBuffer_BP1[N_BP1 + 1] = {0};
//int inputBuffer_BP2[N_BP2 + 1] = {0};

//fix coef arrays
int b_LP_Fix[N_LP + 1] = {0};
int b_HP_Fix[N_HP + 1] = {0};
int b_BP1_Fix[N_BP1 + 1] = {0};
int b_BP2_Fix[N_BP2 + 1] = {0};

const int fractionalBits = 10;
const int scaleFactor = 1 << fractionalBits; // 2^10 = 1024

void bufferShift(int* inputBuffer, int N, int input) {
  for (int i = N; i > 0; i--) {
    inputBuffer[i] = inputBuffer[i - 1];
  }
  inputBuffer[0] = input;
}

float applyFIRFilter(int* b, int* inputBuffer, int N) {
  long output = 0;
  for (int i = 0; i <= N; i++) {
    long product = (long)inputBuffer[i] * b[i];
    output += product;
  }
  return (float)output / scaleFactor;
}

unsigned long lastTime_HP = 0, lastTime_BP1 = 0, lastTime_BP2 = 0, lastTime_LP = 0;
float maxAmplitude_HP = 0, maxAmplitude_BP1 = 0, maxAmplitude_BP2 = 0, maxAmplitude_LP = 0;
// Your detectAmplitude function here

// Function to detect the peak amplitude of a signal over a specified time window
float detectAmplitude(int input, unsigned long& lastTime, float& maxAmplitude) {
  float rectified = abs(input);                 // Rectify the input signal
  static const unsigned long windowSize = 200;  // Define the window size in milliseconds

  unsigned long currTime = millis();
  if (currTime - lastTime > windowSize) {
    // Window has elapsed, time to report and reset
    lastTime = currTime;                 // Update the last time for the next window
    float peakAmplitude = maxAmplitude;  // Store the peak amplitude for reporting
    maxAmplitude = 0;                    // Reset the peak amplitude for the next measurement window
    return peakAmplitude;                // Return the detected peak amplitude for this window
  } else {
    // Window has not yet elapsed, update the peak amplitude if necessary
    if (rectified > maxAmplitude) {
      maxAmplitude = rectified;  // New peak amplitude found
    }
  }
  return maxAmplitude;  // Return the current peak amplitude (if the window hasn't elapsed yet)
}

byte fullBlock[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte emptyBlock[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

void printToScreen(int pos1, int pos2, float nivoo) {
  if (nivoo < 0.25) {
    lcd.setCursor(pos1, pos2);
    //int numCharsToClear = lcd.numCols() - 3; // Assuming the LCD has 16 columns, adjust accordingly
    //
    //// Clear each character from the current cursor position to the end of the line
    for (int i = 0; i < 4; i++) {
      lcd.print(" ");  // Print a space to clear each character
    }
  } else if (nivoo > 0.9) {
    lcd.setCursor(pos1, pos2);
    lcd.write(byte(1));
    lcd.write(byte(1));
    lcd.write(byte(1));
    lcd.write(byte(1));
  } else if (nivoo > 0.75) {
    lcd.setCursor(pos1, pos2);
    lcd.write(byte(1));
    lcd.write(byte(1));
    lcd.write(byte(1));
  } else if (nivoo > 0.5) {
    lcd.setCursor(pos1, pos2);
    lcd.write(byte(1));
    lcd.write(byte(1));
  } else if (nivoo >= 0.25) {
    lcd.setCursor(pos1, pos2);
    lcd.write(byte(1));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  DDRD |= (1 << DDD6);
  ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2));     //clear ADC prescaler bits
  ADCSRA |= 0 << (ADPS2) | 1 << (ADPS1) | 0 << (ADPS0);  //set prescaler to 4
  
  lcd.begin(16, 2);                                      // set up the LCD's number of columns and rows
  //lcd.print("Filter Levels"); // Print a startup message
  lcd.createChar(1, fullBlock);
  lcd.createChar(0, emptyBlock);
  lcd.setCursor(0, 0);
  lcd.write("LPF");
  lcd.setCursor(7, 0);
  lcd.print("BP1F");
  lcd.setCursor(0, 1);
  lcd.write("HPF");
  lcd.setCursor(7, 1);
  lcd.print("BPF2");

  //convet float coef to fixed coef
  for(int i = 0; i <= N_LP; i++){
    b_LP_Fix[i] = round(b_LP[i]*scaleFactor);
    b_HP_Fix[i] = round(b_HP[i]*scaleFactor);
    b_BP1_Fix[i] = round(b_BP1[i]*scaleFactor);
    b_BP2_Fix[i] = round(b_BP2[i]*scaleFactor);
    Serial.print(b_LP_Fix[i]);
    Serial.print("\n");
  };
}

unsigned long prevTime = 0;
unsigned long prevTime2 = 0;
unsigned long prevTime3 = 0;
float amplitude_LP = 0;
float amplitude_HP = 0;
float amplitude_BP1 = 0;
float amplitude_BP2 = 0;
float filtered_LP = 0;
float filtered_HP = 0;
float filtered_BP1 = 0;
float filtered_BP2 = 0;

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = micros();

  PORTD ^= (1 << PORTD6);

  int rawValue = analogRead(A0) - 400;
  int voltage = (rawValue * (1.0 / 500.0))*scaleFactor; //times 1024, to convert into fixed point
  
  if ((currentTime - prevTime) > 400) { //muuda siin aeg filtrite sampling ratega(hetkel 400us on siis 2.5kHz)
    
    filtered_LP = applyFIRFilter(b_LP_Fix, inputBuffer, N_LP);
    filtered_HP = applyFIRFilter(b_HP_Fix, inputBuffer, N_HP);
    filtered_BP1 = applyFIRFilter(b_BP1_Fix, inputBuffer, N_BP1);
    filtered_BP2 = applyFIRFilter(b_BP2_Fix, inputBuffer, N_BP2);
    
    bufferShift(inputBuffer, N_LP, voltage);

    prevTime = currentTime;
  }

  currentTime = micros();
  
  if ((currentTime - prevTime2) > 100000) {
  
    amplitude_LP = detectAmplitude(filtered_LP, lastTime_LP, maxAmplitude_LP);
    amplitude_HP = detectAmplitude(filtered_HP, lastTime_HP, maxAmplitude_HP);
    amplitude_BP1 = detectAmplitude(filtered_BP1, lastTime_BP1, maxAmplitude_BP1);
    amplitude_BP2 = detectAmplitude(filtered_BP2, lastTime_BP1, maxAmplitude_BP1);
    
    prevTime2 = currentTime;
  }
  
  currentTime = micros();
  
  if ((currentTime - prevTime3) > 200000) {
    
    float amplitude_LP_float = amplitude_LP / (float)scaleFactor;
    float amplitude_HP_float = amplitude_HP / (float)scaleFactor;
    float amplitude_BP1_float = amplitude_BP1 / (float)scaleFactor;
    float amplitude_BP2_float = amplitude_BP2 / (float)scaleFactor;
  
    printToScreen(3, 0, amplitude_LP_float * 2.5);
    printToScreen(3, 1, amplitude_HP_float * 2.5);
    printToScreen(11, 0, amplitude_BP1_float * 2.5);
    printToScreen(11, 1, amplitude_BP2_float * 2.5);
    prevTime3 = currentTime;
  }
}