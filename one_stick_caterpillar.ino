/* -----------------------------------------------------------------------------
 * Copyright 2017 E. Kooistra
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ---------------------------------------------------------------------------- */

/*
 * Author: E. Kooistra
 *   4 mar 2018 First version
 *   2 may 2018 Added and verified schemes to control L,R
 *   3 may 2019 Added and verified schemes to use XY direct or mapped L,R. 
 *              Use onestick.h.
 *  30 may 2019 Corrected polar45 scaling.
 *  30 dec 2020 Added polar90 and verified on Gnoe
 * Purpose: 
 *   Mix Radio Control (RC) channels 1 and 2 of the RC stick to control caterpillar tracks 
 *   with one stick.
 * Description:
 * . Measure RC Rx channel PWM pulse width using interrupt on pin change for all 2 Rx channels 
 *   on PIN_CH1 and PIN_CH2. Ch1 = x and Ch2 = y.
 * . Analogue inputs ADC_CH1 and ADC_CH2 provide alternative control of Ch1 and Ch2 via 10k 
 *   potmeters for test purposes without RC.
 * . DIP switch to right is open for input high, to left is short to GND for input low.
 *   - DIP4_USE_RC low selects using potmeter control, high selects RC control.
 *   - DIP1_SCHEME, DIP2_SCHEME, DIP3_SCHEME select mapping scheme from x,y of stick to L,R of tracks.
 * . Use ALPHA << 1 to low pass filter the channel for smoother and slower stick response.
 * . With exponentXY > 1 the x an y can be made exponential instead of linear with the stick angle.
 * . For debug purposes SCHEME_DIRECT_XY can be used to apply x,y directly to steer the L,R tracks. 
 *   Hence, using SCHEME_DIRECT_XY is similar to directly connecting the RC receiver CH1, CH2 to 
 *   the ESC1, ESC2.
 * . If the RC transmitter and RC receiver are on and the rxWidth are updated correctly, then 
 *   rxActive = true and then the ESC are controlled by the stick, else the ESC are set to 
 *   stop the motors. If only the RC receiver is on, then it sends default pulses, so then 
 *   rxActive = true.
 * . Generate PWM servo pulse using the Timer1 with Servo.h for the two ESC via PIN_ESC1 left 
 *   and PIN_ESC2 right.
 * . Wire txWidthEsc to rxWidth when DIP4_USE_RC = 0 to measure txWidthEsc with ADC potmeter control.
 * . Wire RC receiver Ch1, Ch2 to rxWidth when DIP4_USE_RC is high to use in application.
 * . Main signal variables:
 *   int16_t rxWidth      : +-ONESTICK_CH_NORM + ONESTICK_CH_CENTER = 1000 - 2000
 *   int16_t chWidth      : +-ONESTICK_CH_NORM + ONESTICK_CH_CENTER = 1000 - 2000
 *   float   chValue      : +-ONESTICK_CH_NORM                      = +-500
 *   float   chFiltered   : +-ONESTICK_CH_NORM                      = +-500
 *   float   x,y          : +-ONESTICK_CH_NORM                      = +-500
 *   float   L,R          : +-ONESTICK_CH_NORM                      = +-500
 *   int16_t txWidthEsc   : +-ONESTICK_CH_NORM + ONESTICK_CH_CENTER = 1000 - 2000
 *   
 */

#include <math.h>
#include <util/atomic.h>  // for atomic variables between loop() and ISR()
#include <Servo.h>
#include <onestick.h>

/***************************************************************/
/* Constants                                                   */
/***************************************************************/

// Pins
#define PIN_V_LIPO        A7  // A7,  used in combination with RC control, not available with potmeter control
#define ADC_CH1           A7  // A7,  used for potmeter control
#define ADC_CH2           A6  // A6,  used for potmeter control
#define DIP4_USE_RC       19  // A5,  left closed short is LOW - right open pull up is HIGH
#define DIP3_SCHEME       18  // A4,  left closed short is LOW - right open pull up is HIGH
#define DIP2_SCHEME       17  // A3,  left closed short is LOW - right open pull up is HIGH
#define DIP1_SCHEME       16  // A2,  left closed short is LOW - right open pull up is HIGH
#define PIN_ESC1          15  // A1
#define PIN_ESC2          14  // A0
#define PIN_STOP          13  // D13
#define PIN_CH1            2  // D2
#define PIN_CH2            3  // D3
#define PIN_LED            4  // D4

#define USE_SERIAL_PRINT   true
#define LED_PERIOD_US      1000000

// Power supply monitor
// Digital VCC = 5V, divider resistors yield
// V_LIPO_MAX = 5000mV * (2k2 + 6k8) / 2k2 = 20454 mV
#define V_LIPO_MAX         20454  // in mV for PIN_V_LIPO
#define V_LIPO_NOMINAL     11100  // in mV for 3S LIPO
#define V_LIPO_CUTOFF       9900  // in mV for 3S LIPO

// Vlipo voltage measurement conditioning
const float LIPO_ALPHA = 0.02;  // IIR filter coefficient > 0 and <= 1

// Two RC channels of one RC stick:
// . array index [0] = Ch1 = X left-right
// . array index [1] = Ch2 = Y up-down
#define N_CH 2
#define SCHEME_POLAR90             5  // DIP1,2,3 = HLH, default
#define SCHEME_POLAR45             4  // DIP1,2,3 = HLL
#define SCHEME_ROTATE45            3  // DIP1,2,3 = LHH
#define SCHEME_BI_DIRECTION_SPEED  2  // DIP1,2,3 = LHL
#define SCHEME_UNI_DIRECTION_SPEED 1  // DIP1,2,3 = LLH
#define SCHEME_DIRECT_XY           0  // DIP1,2,3 = LLL, for debugging purposes, similar to directly connecting the RC receiver
                                      //                 CH1, CH2 to the ESC1, ESC2.

// Input channel 1, 2, conditioning parameters
const float CH_ZONE_IN = 20;    // Control Ch dead zone force output to 0 for input between +-CH_ZONE_IN
const float CH_ZONE_OUT = 20;   // Control Ch set ouput at +-CH_ZONE_OUT when input at +-CH_ZONE_IN, keep same full scale range
const float ALPHA = 0.05;       // IIR filter coefficient, choose 0.05 for sufficient low pass while maintaining fast stop
const float SKEW = 0.0;         // Optional skew for rotate45 xy --> LR mapping scheme

#define USE_DUAL_SLOPE true
const float LINEAR_SLOPE_XP            = 0.5;  // Xp coordinate for dual slope linear channel response when USE_DUAL_SLOPE
const float EXPONENT_X_POLAR90         = 2.0;  // Exponential for Ch1 = X per scheme
const float EXPONENT_Y_POLAR90         = 2.0;  // Exponential for Ch2 = Y per scheme
const float EXPONENT_X_POLAR45         = 2.0;
const float EXPONENT_Y_POLAR45         = 2.0;
const float EXPONENT_X_ROTATE45        = 2.0;
const float EXPONENT_Y_ROTATE45        = 2.0;
const float EXPONENT_X_BI_DIRECTION_SPEED  = 2.0;
const float EXPONENT_Y_BI_DIRECTION_SPEED  = 2.0;
const float EXPONENT_X_UNI_DIRECTION_SPEED = 3.0;
const float EXPONENT_Y_UNI_DIRECTION_SPEED = 2.0;
const float EXPONENT_X_DIRECT_XY       = 1.0;
const float EXPONENT_Y_DIRECT_XY       = 1.0;

// Use CALIBRATE to adjust input and output offsets
const int16_t TX_CALIBRATE_LR[] = {0, 0};  // use 0, 0 with RC
                                           // measured rxWidth 12,8 and 4,0 --> compensate average about -10,-3
                                           // measured using ADC potmeter control and wire txWidthEsc to rxWidth

/***************************************************************/
/* Global volatile variables that are used in loop() and ISR() */
/***************************************************************/
// . Receive PWM
volatile int16_t rxWidth[N_CH];
volatile uint16_t rxCnt[N_CH];

/***************************************************************/
/* Global variables that are used in both setup() and loop()   */
/***************************************************************/
// Generate PWM via Servo Timer0 for ESC left track and ESC right track
Servo txPwmEsc[N_CH];

/***************************************************************/
/* Local functions                                             */
/***************************************************************/

/*************************************************************/
/* The setup routine runs once when you press reset:        */
/*************************************************************/
void setup() {
  int i;
  
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Receive channel PWM via pin change interrupt on port D
  pinMode(PIN_CH1, INPUT_PULLUP);      // D2 input pull up default to avoid random Rx events when there is no PWM input
  pinMode(PIN_CH2, INPUT_PULLUP);      // D3 input pull up default to avoid random Rx events when there is no PWM input
  PCICR |= (1 << PCIE2);     // set PCIE2 to enable PCMSK2 scan for port D
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 digital input D2 to trigger an interrupt on state change = Ch1
  PCMSK2 |= (1 << PCINT19);  // set PCINT19 digital input D3 to trigger an interrupt on state change = Ch2

  // DIP switch selection inputs
  pinMode(DIP4_USE_RC, INPUT_PULLUP);      // input pull up default selects RC control, connect to GND for potmeter control
  pinMode(DIP3_SCHEME, INPUT_PULLUP);      // XY --> LR scheme
  pinMode(DIP2_SCHEME, INPUT_PULLUP);      // XY --> LR scheme
  pinMode(DIP1_SCHEME, INPUT_PULLUP);      // XY --> LR scheme

  // LED output
  pinMode(PIN_LED, OUTPUT);  // D4

  // Initialize servos that use Timer1
  txPwmEsc[0].attach(PIN_ESC1);   // [0] = A1 = ESC1 is left
  txPwmEsc[1].attach(PIN_ESC2);   // [1] = A0 = ESC2 is rigth
}


/*************************************************************/
/* The loop routine runs over and over again forever:        */
/*************************************************************/
void loop() {
  /* Loop state static variables that remain after each loop */
  // . LED status
  static bool ledGreen = false;
  static uint32_t led_us = 0;
  // . loop status
  static uint32_t begin_us, end_us, hold_begin_us;
  static uint16_t loopCnt = 0;
  // . Lipo 3S accu status
  static int16_t vLipo = V_LIPO_NOMINAL;  // measured LIPO voltage
  static float vLipoFiltered = float(V_LIPO_NOMINAL);  // filtered LIPO voltage measurement
  // . Rx status
  static uint16_t prevRdCnt[N_CH] = {0, 0};
  static bool rxActive = false;
  static bool chCalibrated;
  static int16_t chMin[N_CH] = {ONESTICK_CH_CENTER, ONESTICK_CH_CENTER};  // minimum measured channel pulse width
  static int16_t chMax[N_CH] = {ONESTICK_CH_CENTER, ONESTICK_CH_CENTER};  // maximum measured channel pulse width
  // . Low pass IIR filter
  static float chFiltered[N_CH];

  /* Loop variables that are only used during a loop */
  // . Input
  int16_t scheme;
  bool chStable;
  int16_t adc[N_CH];          // [0] = ADC_CH1, [1] = ADC_CH2
  uint16_t rdCnt[N_CH];       // [0:1] = Ch1, Ch2
  int16_t rdWidth[N_CH];      // [0:1] = Ch1, Ch2
  int16_t chWidth[N_CH];      // [0:1] = Ch1, Ch2, measured channel pulse width
  float chValue[N_CH];
  float exponentXY[N_CH] = {1.0, 1.0};
  // . Mixer
  ONESTICK_T_TwoDimFloat XY, LR;
  float x, y;                 // Ch1, Ch2
  float L, R;                 // left, rigth track
  // . Output
  bool escEnable;
  int16_t txWidthEsc[N_CH];   // [0] = left, [1] = rigth
  uint16_t i;

  //****************************************************************************
  //* Time measurements to determine micro controller load
  //****************************************************************************
  
  // Timestamps begin_us and end_us are used and logged in the next loop, therefore use
  // hold_begin_us to hold begin_us of this loop.
  begin_us = hold_begin_us;
  hold_begin_us = micros();

  //****************************************************************************
  //* Read input pins at start of the loop
  //****************************************************************************

  if (digitalRead(DIP4_USE_RC)==HIGH) {
    /* RC control */
    // Capture measured Lipo voltage
    vLipo = map(analogRead(PIN_V_LIPO), 0, ONESTICK_ADC_MAX, 0, V_LIPO_MAX);
  } else {
    /* Potmeter control */
    // read ADC input on analog pins
    adc[0] = analogRead(ADC_CH1);  // [0] = ADC_CH1, takes about 114 us
    adc[1] = analogRead(ADC_CH2);  // [1] = ADC_CH2, takes about 114 us
  }

  // Read DIP switch setting for stick control scheme = 1,2,3
  scheme = digitalRead(DIP1_SCHEME) * 4 +
           digitalRead(DIP2_SCHEME) * 2 +
           digitalRead(DIP3_SCHEME);
  switch (scheme) {
    case SCHEME_POLAR90:
      exponentXY[0] = EXPONENT_X_POLAR90;
      exponentXY[1] = EXPONENT_Y_POLAR90;
      break;
    case SCHEME_POLAR45:
      exponentXY[0] = EXPONENT_X_POLAR45;
      exponentXY[1] = EXPONENT_Y_POLAR45;
      break;
    case SCHEME_ROTATE45:
      exponentXY[0] = EXPONENT_X_ROTATE45;
      exponentXY[1] = EXPONENT_Y_ROTATE45;
      break;
    case SCHEME_BI_DIRECTION_SPEED:
      exponentXY[0] = EXPONENT_X_BI_DIRECTION_SPEED;
      exponentXY[1] = EXPONENT_Y_BI_DIRECTION_SPEED;
      break;
    case SCHEME_UNI_DIRECTION_SPEED:
      exponentXY[0] = EXPONENT_X_UNI_DIRECTION_SPEED;
      exponentXY[1] = EXPONENT_Y_UNI_DIRECTION_SPEED;
      break;
    case SCHEME_DIRECT_XY:
      exponentXY[0] = EXPONENT_X_DIRECT_XY;
      exponentXY[1] = EXPONENT_Y_DIRECT_XY;
      break;
    default:  // SCHEME_POLAR90
      exponentXY[0] = EXPONENT_X_POLAR90;
      exponentXY[1] = EXPONENT_Y_POLAR90;
      break;
  }

  //****************************************************************************
  //* Default assume the ESC motors can be enabled
  //****************************************************************************
  escEnable = true;
  
  //****************************************************************************
  //* Read ISR global variables at one instant at start of the loop
  //****************************************************************************

  // from ISR() to loop()
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Capture the RC input channels:
    for (i=0; i<N_CH; i++) {
      rdWidth[i] = rxWidth[i];
      rdCnt[i] = rxCnt[i];
    }
  }
  
  //****************************************************************************
  //* Check Rx is active on PIN_CH1, PIN_CH2
  //****************************************************************************
  
  // Detect whether the Rx input rxWidth from the RC is active
  // . Rx is active when the channels from the RC transmitter are connected to PIN_CH1,
  //   PIN_CH2 or when the txPwmEsc[] are wired to the PIN_CH1, PIN_CH2
  // . Only update rxActive status once every few loops, because the loop rate and the
  //   Rx rate are both about 50 Hz, else no check and keep rxActive status
  if (loopCnt % 5 == 0) {
    rxActive = ONESTICK_check_rx_active(rdCnt, prevRdCnt, rdWidth, N_CH);
  }


  //****************************************************************************
  //* Capture one stick input: Ch1, Ch2
  //****************************************************************************
  
  // Select between RC control or ADC potmeter control
  if (digitalRead(DIP4_USE_RC)==HIGH) {
    /* RC control */
    for (i=0; i<N_CH; i++) {
      // use RC input that was received by ISR()
      chWidth[i] = rdWidth[i];
    }
  } else {
    /* Potmeter control */
    // map ADC range to servo pulse range
    for (i=0; i<N_CH; i++) {
      chWidth[i] = map(adc[i], 0, ONESTICK_ADC_MAX, ONESTICK_CH_WIDTH_MIN, ONESTICK_CH_WIDTH_MAX);
    }
  }


  //****************************************************************************
  //* Calibrate pulse width per channel
  //****************************************************************************

  if (rxActive) {
    // For initial calibration first all channels must have been put in their extreme full scale
    // (stick) positions. Then all channels have to be put at their center position. After that
    // inital full scale calibration is achieved and the channels can be used for control.
    // During control the full scale min, max extremes per channel are updated by the calibration,
    // in case the full scale range widens.
    chCalibrated = ONESTICK_calibrate_rx_min_max(chWidth, chMin, chMax, N_CH);
    #if USE_SERIAL_PRINT
      if (chCalibrated) {
        if (loopCnt % 500 == 0) {
          for (i=0; i<N_CH; i++) {
            Serial.print("Calibrated ");
            Serial.print(i);
            Serial.print(": chWidth=");
            Serial.print(chWidth[i]);
            Serial.print(", ");
            Serial.print(chWidth[i] - (chMin[i] + chMax[i]) / 2);
            Serial.print(" chMin=");
            Serial.print(chMin[i]);
            Serial.print(" chMax=");
            Serial.print(chMax[i]);
            Serial.println("");
          }
        }
      }
    #endif  // USE_SERIAL_PRINT
  }
  // else: Remain in calibrated state if rxActive goes low, because full scale calibration
  //       can only safely be done at initial power up

  // Only use the chWidth pulse if it is active and calibrated
  chStable = rxActive && chCalibrated;
  if (chStable) {
    // apply calibration to pulse width when Rx is active and full scale calibration was done at least once
    for (i=0; i<N_CH; i++) {
      chWidth[i] = map(chWidth[i], chMin[i], chMax[i], ONESTICK_CH_WIDTH_MIN, ONESTICK_CH_WIDTH_MAX);
    }
  } else {
    // Disable ESC motors when RC is not stable
    escEnable = false;
    
    // use default nominal center pulse width
    for (i=0; i<N_CH; i++) {
      chWidth[i] = ONESTICK_CH_CENTER;
    }
  }
  
  //****************************************************************************
  //* Filter Lipo voltage and disable ESC motors if Lipo battery is too low
  //****************************************************************************
  // Low pass IIR filter vLipo voltage
  vLipoFiltered = ONESTICK_low_pass_filter(float(vLipo), vLipoFiltered, LIPO_ALPHA);
  if (vLipoFiltered < V_LIPO_CUTOFF) {
    // disable ESC motors when V_lipo is too low, to have lowest power consumption
    escEnable = false;
  }
  
  // Stop motors if PIN_STOP is high (due to button)
  if (digitalRead(PIN_STOP)==HIGH) {
    escEnable = false;
  }
  
  if (!escEnable) {
    // use default nominal center pulse width
    for (i=0; i<N_CH; i++) {
      chWidth[i] = ONESTICK_CH_CENTER;
    }
  }

  
  //****************************************************************************
  //* Condition one stick input: Ch1, Ch2 --> x,y
  //****************************************************************************

  // Inverting Ch1 swaps L and R.
  // Use invert Ch1 = X stick control, because turning left requires right track to run faster.
  chWidth[0] = ONESTICK_invert_channel_pulse(chWidth[0], true);

  for (i=0; i<N_CH; i++) {
    // Use chValue in +-500 range
    chValue[i] = (float)ONESTICK_remove_channel_pulse_offset(chWidth[i]);
    
    // Apply dead spot to dual channel of stick
    if (i % 2 == 1) {
      XY.a = chValue[i-1];
      XY.b = chValue[i];
      XY = ONESTICK_create_dual_channel_dead_spot(XY, CH_ZONE_IN);
      chValue[i-1] = XY.a;
      chValue[i] = XY.b;
    }
    
    // Low pass IIR filter channel with 0 < ALPHA < 1.0, when ALPHA = 1.0 then no effect
    chFiltered[i] = ONESTICK_low_pass_filter(chValue[i], chFiltered[i], ALPHA);

    // Create linear dual slope or create exponential channel using pow() function
#if USE_DUAL_SLOPE
    chValue[i] = ONESTICK_create_dual_slope_channel(chFiltered[i], LINEAR_SLOPE_XP, pow(LINEAR_SLOPE_XP, exponentXY[i]));
#else
    chValue[i] = ONESTICK_create_exponential_channel(chFiltered[i], exponentXY[i]);
#endif
  }

  // Map ch1, ch2 to x, y with range +-ONESTICK_CH_NORM = +-500
  x = chValue[0];  // ch1 range +-ONESTICK_CH_NORM = +-500
  y = chValue[1];  // ch2 range +-ONESTICK_CH_NORM = +-500

  
  //****************************************************************************
  //* Map one stick input x,y --> L,R track control
  //****************************************************************************
  
  // Mix one stick ch1, ch2 into track left and right using rotate45 or direction_speed scheme
  switch (scheme) {
    case SCHEME_POLAR90:
      LR = ONESTICK_onestick_polar90(x, y);                // Rotated XY stick control using polar coordinates
      break;
    case SCHEME_POLAR45:
      LR = ONESTICK_onestick_polar45(x, y);                // Rotated XY stick control using polar coordinates
      break;
    case SCHEME_ROTATE45:
      LR = ONESTICK_onestick_rotate45(x, y, SKEW);         // Rotated XY stick control using linearized mapping
      break;
    case SCHEME_BI_DIRECTION_SPEED:
      LR = ONESTICK_onestick_direction_speed(x, y, true);  // Speed Y and bi direction X stick control
      break;
    case SCHEME_UNI_DIRECTION_SPEED:
      LR = ONESTICK_onestick_direction_speed(x, y, false); // Speed Y and uni direction X stick control
      break;
    case SCHEME_DIRECT_XY:
      LR.a = x;
      LR.b = y;
      break;
    default:  // SCHEME_POLAR90
      LR = ONESTICK_onestick_polar90(x, y);                // Rotated XY stick control using polar coordinates
      break;
  }
  L = LR.a;
  R = LR.b;
  

  //****************************************************************************
  //* Output L,R track control to ESC motor of the tracks
  //****************************************************************************

  // Control ESC via Timer1, [0] is left track, [1] is rigth track
  // . Default set ESC at zero speed
  txWidthEsc[0] = ONESTICK_CH_CENTER + TX_CALIBRATE_LR[0];
  txWidthEsc[1] = ONESTICK_CH_CENTER + TX_CALIBRATE_LR[1];
  // . Add L,R to L,R ESC control if escEnable
  if (escEnable) {
    txWidthEsc[0] += (int16_t)roundf(L);   // Ch1 via L
    txWidthEsc[1] += (int16_t)roundf(R);   // Ch2 via R
  }
  // Write L,R control to ESC
  txPwmEsc[0].write(txWidthEsc[0]);
  txPwmEsc[1].write(txWidthEsc[1]);


  //****************************************************************************
  //* Control green status LED
  //****************************************************************************
  
  if (begin_us > led_us + LED_PERIOD_US/2) {
    digitalWrite(PIN_LED, ledGreen);
    //ONESTICK_control_rx_status_led(rxActive, chCalibrated, &ledGreen);
    ONESTICK_control_status_led(escEnable, true, &ledGreen); // blink when enabled, else off
    led_us = begin_us;
    /*
    #if USE_SERIAL_PRINT
      Serial.print(", rxActive=");
      Serial.print(rxActive);
      Serial.print(" chCalibrated=");
      Serial.print(chCalibrated);
      Serial.print(" escEnable=");
      Serial.print(escEnable);
      Serial.print(", ledGreen=");
      Serial.println(ledGreen);
    #endif  // USE_SERIAL_PRINT
    */
  }


  //****************************************************************************
  //* Log to serial monitor on PC via UART-USB
  //****************************************************************************
  
  #if USE_SERIAL_PRINT
    if (loopCnt % 10 == 0) {
      // Print the measured time spend processing and printing in previous loop, so without the delay()
      //Serial.println(end_us-begin_us);
    }
    if (loopCnt % 50 == 0) {
      if (digitalRead(DIP4_USE_RC)==HIGH) {
        /* RC control */
        // Print the Rx channel width measured values and the L,R mixed values
        Serial.print(" Rx1=");
        Serial.print(rdWidth[0]);  // rdWidth from RC channels or wired txWidthEsc channels
        Serial.print(" Rx2=");
        Serial.print(rdWidth[1]);
        Serial.print("  Ch1=");
        Serial.print(chWidth[0]);  // chWidth = x,y
        Serial.print(" Ch2=");
        Serial.print(chWidth[1]);
        Serial.print(" ");
        Serial.print(scheme);
        Serial.print(" ");
        Serial.print(escEnable);
        Serial.print("  Tx0=");
        Serial.print(txWidthEsc[0]);
        Serial.print(" Tx1=");
        Serial.print(txWidthEsc[1]);
        Serial.print(" vLipo=");
        Serial.print(vLipo);
        Serial.println("");
      } else {
        /* Potmeter control */
        // Print the ch width measured values and the L,R mixed values
        Serial.print(" Ch1=");
        Serial.print(chWidth[0]);  // chWidth = x,y from ADC potmeter
        Serial.print(" Ch2=");
        Serial.print(chWidth[1]);
        Serial.print("  L=");
        Serial.print(L);           // map x,y to L,R
        Serial.print(" R=");
        Serial.print(R);
        Serial.print("  Tx0=");
        Serial.print(txWidthEsc[0]);  // map x,y to L,R for txWidthEsc
        Serial.print(" Tx1=");
        Serial.print(txWidthEsc[1]);
        Serial.print("  Rx1=");
        Serial.print(rdWidth[0]);  // connect txWidthEsc[0] to rdWidth[0] with female-female wire and DIP4_USE_RC = false
        Serial.print(" Rx2=");
        Serial.print(rdWidth[1]);  // connect txWidthEsc[1] to rdWidth[1] with female-female wire and DIP4_USE_RC = false
        Serial.println("");
      }
    }
  #endif  // USE_SERIAL_PRINT

  //****************************************************************************
  //* Maintain loop period
  //****************************************************************************
  
  loopCnt++;
  end_us = micros();

  // Main loop() without delay() measured including USE_SERIAL_PRINT, from begin_us to end_us takes:
  //  ~2.8 ms : SCHEME_POLAR90
  //  ~1.8 ms : SCHEME_ROTATE45
  //  ~1.4 ms : SCHEME_BI_DIRECTION_SPEED
  //  ~1.1 ms : SCHEME_DIRECT_XY
  //  ~0.6 ms : After reset
  // Wait 17 ms, to loop at about 50 Hz typical servo pulse rate
  delay(17);
}


/***************************************************************/
/* Interrupt Service Routines (ISR)                             */
/***************************************************************/

/*-----------------------------------------------------------------------------
 * Function name: ISR(PCINT2_vect) for port D
 * Defines: N_CH       = number of channels, pins supported in the array
 * Input REGs: PIND    = read port D pins
 * Globals: rxWidth[]  = measured channel pulse width per pin
 *          rxCnt[]    = count number of pulse per pin
 * Description:
 *   See description of ONESTICK_rx_pulse_width(). 
 *   Based on similar PCINT ISR() by Joop Brokking in YMFC-3D_V2.
 */
ISR(PCINT2_vect) {
  static uint8_t prevPinLevel[N_CH];
  static uint32_t rxStartUs[N_CH];
  uint8_t pinLevel[N_CH];
  uint32_t currentTimeUs = micros();
  uint16_t i;

  // Use N_CH port pins via pinLevel[]
  pinLevel[0] = PIND & B00000100;   // PIN_CH1 = D2, PCINT18
  pinLevel[1] = PIND & B00001000;   // PIN_CH2 = D3, PCINT19

  // Measure pulse width per input pin using pinLevel[] and currentTimeUs
  for (i=0; i<N_CH; i++) {
    ONESTICK_measure_rx_pulse_width(pinLevel[i],
                                    currentTimeUs,
                                    &prevPinLevel[i],
                                    &rxStartUs[i],
                                    &rxCnt[i],
                                    &rxWidth[i]);
  }
}
