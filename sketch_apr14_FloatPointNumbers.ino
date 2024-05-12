#include <LiquidCrystal.h>
#include <Arduino.h>
// Initialize the LiquidCrystal library with the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int N_LP = 10;
const int N_HP = 10;
const int N_BP1 = 16;
const int N_BP2 = 16;

float b_LP[N_LP + 1] = {
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
    0.000016239357898861,
    -0.000165720831801527,
    0.000651927273419365,
    -0.000577546516239022,
    -0.010453303779118277,
    0.052421250164514391,
    -0.057381259411664984,
    -0.117227496603579479,
    0.265431820693148968,
    -0.117227496603580644,
    -0.057381259411665601,
    0.052421250164514543,
    -0.010453303779118309,
    -0.000577546516239013,
    0.000651927273419357,
    -0.000165720831801566,
    0.000016239357898947,
};

float inputBuffer[N_LP + 1] = {0};
//float inputBuffer_HP[N_HP + 1] = {0};
//float inputBuffer_BP1[N_BP1 + 1] = {0};
//float inputBuffer_BP2[N_BP2 + 1] = {0};

void bufferShift(float* inputBuffer, int N, float input) {
  for (int i = N; i > 0; i--) {
    inputBuffer[i] = inputBuffer[i - 1];
  }
  inputBuffer[0] = input;
}

float applyFIRFilter(float* b, float* inputBuffer, int N) {
  float output = 0.00000;
  for (int i = 0; i <= N; i++) {
    output += inputBuffer[i] * b[i];
  }
  return output;
}

unsigned long lastTime_HP = 0, lastTime_BP1 = 0, lastTime_BP2 = 0, lastTime_LP = 0;
float maxAmplitude_HP = 0, maxAmplitude_BP1 = 0, maxAmplitude_BP2 = 0, maxAmplitude_LP = 0;
// Your detectAmplitude function here

// Function to detect the peak amplitude of a signal over a specified time window
float detectAmplitude(float input, unsigned long& lastTime, float& maxAmplitude) {
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
  float voltage = rawValue * (1.0 / 500.0);
  
  if ((currentTime - prevTime) > 400) { //muuda siin aeg filtrite sampling ratega(hetkel 400us on siis 2.5kHz)
    filtered_LP = applyFIRFilter(b_LP, inputBuffer, N_LP);
    filtered_HP = applyFIRFilter(b_HP, inputBuffer, N_HP);
    filtered_BP1 = applyFIRFilter(b_BP1, inputBuffer, N_BP1);
    filtered_BP2 = applyFIRFilter(b_BP2, inputBuffer, N_BP2);
    
    bufferShift(inputBuffer, N_LP, voltage);
    
    prevTime = currentTime;
  }

  currentTime = micros();
  
  if ((currentTime - prevTime2) > 100000) {
    //Serial.println(filtered_LP);
    //Serial.print(" , ");
  
    amplitude_LP = detectAmplitude(filtered_LP, lastTime_LP, maxAmplitude_LP);
    amplitude_HP = detectAmplitude(filtered_HP, lastTime_HP, maxAmplitude_HP);
    amplitude_BP1 = detectAmplitude(filtered_BP1, lastTime_BP1, maxAmplitude_BP1);
    amplitude_BP2 = detectAmplitude(filtered_BP2, lastTime_BP2, maxAmplitude_BP2);
  
    prevTime2 = currentTime;
  }
  
  currentTime = micros();

  if ((currentTime - prevTime3) > 200000) {
    //Serial.println(amplitude_LP * 2.5);
    //Serial.print(" , ");
    printToScreen(3, 0, amplitude_LP * 2.5);
    printToScreen(3, 1, amplitude_HP * 2.5);
    printToScreen(11, 0, amplitude_BP1 * 2.5);
    printToScreen(11, 1, amplitude_BP2 * 2.5);
    prevTime3 = currentTime;
  }
}