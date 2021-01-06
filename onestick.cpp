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
 * -----------------------------------------------------------------------------
 */

/* Compile switches. */

/* includes */
#include <math.h>
#include "onestick.h"  // use onestick.cpp instead of c to be able to use const instead of #define

/* private constants (#define) */
#define CH_SHIFT       20   // assume RC pulse center is within ONESTICK_CH_CENTER +- CH_SHIFT
#define CH_BACKOFF     200  // assume RC pulse range is at least +-(ONESTICK_CH_NORM + CH_BACKOFF)
#define CH_MIN_LO     (ONESTICK_CH_WIDTH_MIN - CH_BACKOFF)
#define CH_MIN_HI     (ONESTICK_CH_WIDTH_MIN + CH_BACKOFF)
#define CH_MAX_LO     (ONESTICK_CH_WIDTH_MAX - CH_BACKOFF)
#define CH_MAX_HI     (ONESTICK_CH_WIDTH_MAX + CH_BACKOFF)
#define CH_CENTER_LO  (ONESTICK_CH_CENTER - CH_SHIFT)
#define CH_CENTER_HI  (ONESTICK_CH_CENTER + CH_SHIFT)

/* private types */

/* private function protos */

/* private constants (const) */

/* private variables */

/* public variables */

/* public function implements */

/*-----------------------------------------------------------------------------
 * Function name: ONESTICK_sin_triangle
 *                ONESTICK_cos_triangle
 *                ONESTICK_sin_square
 *                ONESTICK_cos_square
 * Parameter: phi = angle in radians
 * Return:    wave(phi) in range [-1, +1] with same phase as sin(phi) or
 *            as cos(phi), for different waveforms (triangle, square)
 * Description:
 *   Dependent on quadrant of phi, return wave(phi) in range -1,+1. 
 * Remark:
 * . Quadrant boundaries are defined such that phi = N * pi/2 belongs to
 *   quadrant N+1.
 */
float ONESTICK_sin_triangle(float phi) {
  float p;
  
  p = fmod(phi, 2*M_PI);  // modulo to get p in range [0, 2pi>
  if (p < 0) {
    p += 2*M_PI;     // adjust for negative phi
  }
  p = p / M_PI_2;    // normalize p per quadrant angle of pi/2
  if (p < 1) {
    return p;
  } else {
    if (p < 3) {
      return 2 - p;
    } else {
      return p - 4;
    }
  }
}

float ONESTICK_cos_triangle(float phi) {
  return ONESTICK_sin_triangle(phi + M_PI_2);
}

float ONESTICK_sin_square(float phi) {
  float p;
  
  p = fmod(phi, 2*M_PI);  // modulo to get p in range [0, 2pi>
  if (p < 0) {
    p += 2*M_PI;     // adjust for negative phi
  }
  p = p / M_PI_2;    // normalize p per quadrant angle of pi/2
  if (p < 2) {
    return 1;
  } else {
    return -1;
  }
}

float ONESTICK_cos_square(float phi) {
  return ONESTICK_sin_square(phi + M_PI_2);
}
    
/*-----------------------------------------------------------------------------
 * Function name: ONESTICK_measure_rx_pulse_width
 * Parameter:
 * - Input
 *   . pinLevel      = pin level of the connected PCINT pin
 *   . currentTimeUs = time at call of this function
 * - Input/output
 *   . pPrevPinLevel = pin level at previous call of this function
 *   . pRxStartUs    = stores currentTimeUs at pulse start time
 *   . pRxCnt        = maintain count of number of pulses
 * - Output
 *   . pRxWidth      = measured pulse width
 * Return: void
 * Description:
 *   The pulse width rxWidth is measured using currentTimeUs at the start and at the end of the
 *   pulse. The function is called from the pin change interrupt (PCINT) interrupt service
 *   routine (ISR) each time there is a pinLevel status change on a connected PCINT pin. For radio
 *   control (RC) channels typically 1000 us <= rxWidth <= 2000 us and the typical pulse rate is
 *   50 Hz. For Rx activity detection the function also counts the number of pulses rxCnt.
 * Usage:
 * . The pRxCnt and pRxWidth have to be volatile in this function, because they are declared as
 *   volatile by caller program, and the function is used in the ISR().
 * . In the setup() the PCINT needs to be enabled, e.g. for pin D2:
 *     pinMode(D2, INPUT_PULLUP);
 *     PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK2 scan for port D
 *     PCMSK2 |= (1 << PCINT18);  // set PCINT18 (D2) to trigger an interrupt on state change
 *   The pullup is needed to avoid spurious interrupts when no signal is connected to the pin.
 * . The PCINT ISR() can use micros() timer to measure currentTimeUs
 * . The PCINT ISR() argument dependents on which pins are actually used:
 *   - PCINT0_vect for PCINT[ 0: 7] via port B
 *   - PCINT1_vect for PCINT[ 8:15] via port C
 *   - PCINT2_vect for PCINT[16:23] via port D
 * . The PCINT ISR() can determine the pinLevel for a pin on port B, C or D by anding the port
 *   register PINB, PINC, PIND with the pin index, e.g. pinLevel = PIND & B00000100 for pin D2,
 *   where PIND is a macro that actually reads the port register.
 * . The PCINT ISR() can only serve pins from the same port, so maximum N_CH = 8 pins. This
 *   function is then called in a for (i=0; i<N_CH; i++) loop. The PCINT ISR() interrupt occurs
 *   when one or multiple pins change state.
 * Acknowledgement:
 *   This function and how it is used in the PCINT ISR() are based on similar PCINT ISR() by 
 *   Joop Brokking in YMFC-3D_V2.
 */
void ONESTICK_measure_rx_pulse_width(uint8_t pinLevel,
                                     uint32_t currentTimeUs,
                                     uint8_t *pPrevPinLevel,
                                     uint32_t *pRxStartUs,
                                     volatile uint16_t *pRxCnt,
                                     volatile int16_t *pRxWidth) {
  if (pinLevel) {
    if (*pPrevPinLevel == 0) {
      // Start of pulse                     // Input is high and changed from 0 to 1
      *pPrevPinLevel = 1;                   // Remember current input level
      *pRxStartUs = currentTimeUs;          // Remember pulse start time
    }
  } else {
    if (*pPrevPinLevel == 1) {
      // End of pulse                           // Input is low and changed from 1 to 0
      *pPrevPinLevel = 0;                       // Remember current input level
      *pRxWidth = currentTimeUs - *pRxStartUs;  // Calculate Rx channel pulse width
      *pRxCnt += 1;                             // Increment pulse count at end of pulse event
    }
  }
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_check_rx_active
 * Parameters:    rxCnt[]     = current PWM pulse interrupt count per RC channel
 *                prevRxCnt[] = previous PWM pulse interrupt count per RC channel.
 *                rxWidth[]   = received PWM pulse width per RC channel, in nominal range 1000 us
 *                              to 2000 us
 *                nofCh       = number of Rx channels to calibrate
 * Return:        active      = true when a valid RC pulse was detected in the last interval for
 *                              all nofCh channels, else false
 * Description:
 *   Check whether all nofCh radio control (RC) channels PWM are active. If the RC was active
 *   since the previous function call then function returns true, else it returns false. The RC
 *   activity is detected based on:
 *   - the rxCnt that should increment each time an PWM pulse interrupt occurred
 *   - the prevRxCnt is used to hold the previous rxCnt
 *   - the rxWidth of the PWM pulses must be in range 1000 us to 2000 us with some margin
 *   The Rx can be active while the transmitter is off, because then the receiver outputs central
 *   pulse width.
 *
 * Remarks:
 * . The prevRxCnt[] is declared by caller and maintained by this function. It cannot be a local
 *   static variable of this function, because the array length is variable.
 * . This function does not work with fail safe RC receiver when transmitter is off, because
 *   the fail safe Rx then still outputs PWM pulses. This function only works if the Rx 
 *   receiver does not output PWM pulse, when the Tx is off.
 * -----------------------------------------------------------------------------
 */
bool ONESTICK_check_rx_active(uint16_t rxCnt[], uint16_t prevRxCnt[], int16_t rxWidth[], uint16_t nofCh) {
  bool active = true;
  uint16_t i;

  for (i=0; i<nofCh; i++) {
    // check whether Rx events occur at all
    if (prevRxCnt[i]==rxCnt[i]) {active = false;}
    prevRxCnt[i] = rxCnt[i];
    // check whether Rx pulse widths are in valid range
    if (rxWidth[i] < CH_MIN_LO) {active = false;}
    if (rxWidth[i] > CH_MAX_HI) {active = false;}
  }

  return active;
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_calibrate_rx_min_max
 * Parameters: chWidth[]  = input received PWM channel pulse width per channel
 *             chMin[]    = output calibrate pulse minimal channel width per received PWM channel
 *             chMax[]    = output calibrate pulse maximum channel width per received PWM channel
 *             nofCh      = number of Rx channels to calibrate
 * Return: calibrated = true when the chMin[] and chMax[] calibration values are valid
 * Description:
 *   The nominal pulse width range is 1000 us to 2000 us, with center value 1500 us. The actual Rx
 *   pulse may have an offset and a scale factor. The calibration measures the actual minimal and
 *   maximal chWidth. The assumption is that the pulse width changes linear between its minimum
 *   and maximum, so the range is symmetrical around the center value. Therefore it is not
 *   necessary to seperately measure and calibrate the center value.
 *   The user should first switch on the RC transmitter, then switch on the RC receiver and
 *   microcontroller. After that the user should move the stick to maximum up-down and left-right,
 *   such that the calibration can measure the full scale chWidth min and max values per channel.
 *   The calibration keeps the smallest min and largest max value for chWidth in chMin and chMax
 *   per channel. The calibration is finished and returns calibrated = true when chMin and chMax
 *   are within hi,lo margin of the nominal full scale range. To declare calibrated = true the
 *   current chWidth value must be within hi,lo margin of the center value to ensure that for all
 *   channels the largest stick extents have been measured. Calibrated can only be declared when
 *   all channels are in center (neutral) position, to avoid unintended control actions just after
 *   calibrated becomes true. Therefore this calibration assumes that the channel has a center
 *   neutral position like a stick control, and cannot be used for e.g. a switch channel that only
 *   has min or max and uses min as neutral or a slider that has min as neutral.
 *   While calibrated = false the stick control should have no effect on the RC controled device.
 *
 *                  ONESTICK_CH_WIDTH_MIN    ONESTICK_CH_WIDTH_MAX
 *                          1000                     2000
 *                        980  1200               1800  2020
 *                  CH_MIN_LO  CH_MIN_HI     CH_MAX_LO  CH_MAX_HI
 *                  <------chMin------->     <------chMax------->
 *
 *   The calibration measurement of chMin and chMax continues also after the calibration has
 *   finished, but the chMin and chMax will then typically not change anymore.
 *   The stick control remains in calibrated state for ever (until reset), because full scale
 *   calibration can only safely be done at initial power up. Hence if the RC transmitter is
 *   switched off and on again, then the stick control will not recalibrate, which is also not
 *   needed.
 * Remarks:
 * . The application should initialize chMin[] and chMax to the nominal center offset value
 *   ONESTICK_CH_CENTER.
 * . The application can calibrate the measured pulse width to the nominal pulse width by means of
 *   map(), e.g:
 *     chWidth = map(chWidth, chMin, chMax, ONESTICK_CH_WIDTH_MIN, ONESTICK_CH_WIDTH_MAX);
 * -----------------------------------------------------------------------------
 */
bool ONESTICK_calibrate_rx_min_max(int16_t chWidth[], int16_t chMin[], int16_t chMax[], uint16_t nofCh) {
  static bool calibrated = false;

  bool calibrationDone = true;
  uint16_t i;

  // Update calibration
  for (i=0; i<nofCh; i++) {
    // hold smallest and largest pulse width that are within range
    if (chWidth[i] > CH_MIN_LO && chWidth[i] < chMin[i]) {chMin[i] = chWidth[i];}
    if (chWidth[i] < CH_MAX_HI && chWidth[i] > chMax[i]) {chMax[i] = chWidth[i];}

    // check whether full scale range has been measured for all channels
    if (chMin[i] > CH_MIN_HI) {calibrationDone = false;}
    if (chMax[i] < CH_MAX_LO) {calibrationDone = false;}

    // check that the current channel pulse is close enough to the center position to declare
    // calibration done
    if (chWidth[i] < CH_CENTER_LO || chWidth[i] > CH_CENTER_HI) {calibrationDone = false;}
  }

  // Enter and remain in calibrated state when calibrationDone has been done at least once
  if (calibrationDone) {
    calibrated = true;
  }

  return calibrated;
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_control_status_led
 * Parameters:
 * - Input
 *   . active     = true when input is active
 *   . calibrated = true when input is calibrated
 * - Output:
 *   . *ledLight = toggle when active and calibrated,
 *                 on     when active but not calibrated,
 *                 off    when not active
 * Description:
 *   Show input status via an LED. The ledLight can be used to control a LED. The blinking
 *   of the LED depends on how often this function is called.
 * -----------------------------------------------------------------------------
 */
void ONESTICK_control_status_led(bool active, bool calibrated, bool *ledLight) {
  if (active) {
    if (calibrated) {
      *ledLight = !*ledLight;  // toggle when Rx is active and calibrated
    } else {
      *ledLight = true;       // constant on when Rx is active, but not calibrated (yet)
    }
  } else {
    *ledLight = false;        // constant off when no Rx
  }
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_invert_channel_pulse
 * Parameters:    chWidth  = channel pulse width in range 1000 us to 2000 us
 *                invert   = when true invert chWidth, else no effect
 * Return:        y        = 3000 us - chWidth when invert else chWidth
 * Description:
 *   Invert channel pulse width in range ONESTICK_CH_WIDTH_MIN - ONESTICK_CH_WIDTH_MAX
 */
int16_t ONESTICK_invert_channel_pulse(int16_t chWidth, bool invert) {
  if (invert) {
    return 2*ONESTICK_CH_CENTER - chWidth;
  } else {
    return chWidth;
  }
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_remove_channel_pulse_offset
 * Parameters:    chWidth  = channel pulse width in range 1000 us to 2000 us
 * Return:        chValue  = channel pulse width in range -500 us to 500 us
 * Description:
 *   Remove ONESTICK_CH_CENTER from servo pulse width to map channel to +-ONESTICK_CH_NORM range.
 */
int16_t ONESTICK_remove_channel_pulse_offset(int16_t chWidth) {
  return chWidth - ONESTICK_CH_CENTER;
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_create_exponential_channel
 * Parameters:    x        = linear channel pulse in range -500 us to 500 us
 *                exponent = when > 1 then more sensitive for small x
 *                           when 1 then no effect, so y = x
 *                           when < 1 then less sensitive for small x
 *                           when 0 then y = clipped x
 * Return:        y        = exponential channel pulse in range -500 us to 500 us
 * Description:
 *   Create exponential channel using pow() function. The input and output fullscale range are
 *   +-ONESTICK_CH_NORM. The exponent provides a single parameter to control the curve between 0
 *   and +-ONESTICK_CH_NORM. For exponent = 1 the curve is linear, for exponent > 0 the curve is
 *   less steep around 0, for exponent < 0 the curve is more steep around 0. For example exponent
 *   2 yields that at half scale the output is at quarter scale, because 0.5**2 = 0.25.
 * -----------------------------------------------------------------------------
 */
float ONESTICK_create_exponential_channel(float x, float exponent) {
  if (x >= 0) {
    return +1 * pow( x, exponent) / pow(ONESTICK_CH_NORM, exponent - 1);
  } else {
    return -1 * pow(-x, exponent) / pow(ONESTICK_CH_NORM, exponent - 1);
  }
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_create_dual_slope_channel
 * Parameters:    x  = linear channel pulse in range -500 us to 500 us
 *                Xp = relative x coordinate in range 0 to 1 of slope point
 *                Yp = relative y coordinate in range 0 to 1 of slope point
 * Return:        y  = dual slope channel pulse in range -500 us to 500 us
 * Description:
 *   Create dual slope channel response for y with slope a = Yp/Xp from (0,0) to (Xp,Yp) and with
 *   slope b = (1-Yp)/(1-Xp) from (Xp,Yp) to 1. Similar for negative y. For Xp is 0 or 1 there is
 *   only one slope and Yp is then an offset:
 *   . If Xp = 0 and Yp > 0 then y ranges from (0,Yp) to (1, 1), so a = 1-Yp
 *   . If Xp = 1 and Yp < 1 then y ranges from (0, 0) to (1,Yp), so a = Yp
 * Remark:
 * . This function was verified using similar function in one_stick_caterpillar.py.
 * -----------------------------------------------------------------------------
 */
float ONESTICK_create_dual_slope_channel(float x, float Xp, float Yp) {
  float a, b;    // slope coefficients
  float xn, yn;  // x normalized, y normalized
  xn = x / ONESTICK_CH_NORM;  // normalize x
  if (Xp <= 0){
    // Single slope with offset +-Yp for xn = 0
    a = 1 - Yp;
    if (xn >= 0) {
	  yn = Yp + a * xn;
    } else {
	  yn = -Yp + a * xn;
    }        
  } else {
	if (Xp >= 1) {
      // Single slope ending at +-Yp for xn = +-1
      a = Yp;
      yn = a * xn;
    } else {
      // Dual slope for yn(xn):
      // . xn < 0 from (-1,-1) to (-Xp,-Yp) and to (0,0)
      // . xn > 0 from (0,0) to (Xp,Yp) and to (1,1)
      a = Yp / Xp;
      b = (1 - Yp) / (1 - Xp);
      if (xn >= -Xp && xn <= Xp) {
	    yn = a * xn;
      } else {
        if (xn >= 0) {
	      yn = Yp + b * (xn - Xp);
        } else {
	      yn = -Yp + b * (xn + Xp);
        }
      }
    }
  }
  return yn * ONESTICK_CH_NORM;  // back to channel pulse range y(x)
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_create_channel_dead_zone
 * Parameters:    x     = input in range +-ONESTICK_CH_NORM
 *                zoneX = input dead zone 0 <= zoneX <= 1 * ONESTICK_CH_NORM
 *                zoneY = output dead zone 0 <= zoneY <= 1 * ONESTICK_CH_NORM
 * Return:        y     = output in range +-ONESTICK_CH_NORM with new dead zone and slope
 * Description:
 *   Set the output dead zone according to the following rules:
 *   - normalized full scale range of input x and output y is +-1
 *   - normalized input dead zone 0 <= zoneX <= 1, normalized output dead zone 0 <= zoneY <= 1
 *   - keep normalized full scale range x = +-1 --> y = +-1
 *   - use ONESTICK_CH_NORM to scale to actual range of x, y, zoneX and zoneY
 *   - force y to 0 in dead zone for -zoneX < x < +zoneX
 *   - force y to +-zoneY at edge of dead zone when x = +-zoneX, keep same full scale range
 *   - linear slope of x is 1 and linear slope of y outside the dead zone is (1-zoneY)/(1-zoneX)
 * Examples:
 *   Typically small values of zoneX and zoneY create a dead zone:
 *   - zoneX = 0.1 and zoneY = 0   : creates dead zone around x = 0
 *   - zoneX = 0   and zoneY = 0.1 : removes dead zone around y = 0
 *   - zoneX = 0.1 and zoneY = 0.1 : forces dead zone y = 0 for x between +-0.1 and otherwise
 *                                   keeps y = x
 *   Using full scale zoneY makes the function act like a clipping function:
 *   - zoneX = 0   and zoneY = 1   : clips y when x != 0, and sets y = 0 when x = 0
 * -----------------------------------------------------------------------------
 */
float ONESTICK_create_channel_dead_zone(float x, float zoneX, float zoneY) {
  float slopeY = (ONESTICK_CH_NORM - zoneY) / (ONESTICK_CH_NORM - zoneX);

  if (x > zoneX) {
    return  zoneY + slopeY * (x - zoneX);
  } else if (x < -zoneX) {
    return -zoneY + slopeY * (x + zoneX);
  } else {
    return 0;
  }
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_create_dual_channel_dead_spot
 * Parameters:    dd     = dual input in range +-ONESTICK_CH_NORM
 *                zoneIn = input dead zone 0 <= zoneIn <= 1 * ONESTICK_CH_NORM
 * Return:        rr     = dual output in range +-ONESTICK_CH_NORM with dead zone
 * Description:
 *   The zoneOut = zoneIn, so slopeOut = slopeIn = 1. Therefor the dead zone
 *   function per channel ONESTICK_create_channel_dead_zone() reduces to:
 *
 *     return (x < -zoneIn || x > zoneIn) ? x : 0;
 *
 *   Set the dual output dead spot according to the following dual channel 
 *   regions:
 *
 *     x < -zoneIn |      |  x > zoneIn
 *              11 |  01  |  11  y > zoneIn
 *     -------------------------------------
 *              10 |  00  |  10
 *     -------------------------------------
 *              11 |  01  |  11  y < -zoneIn
 *
 *               band
 *     Region    XY
 *         0  =  00 :  a = 0, b = 0, in dead spot
 *         1  =  01 :  a = 0, b = y
 *         2  =  10 :  a = x, b = 0
 *         3  =  11 :  a = x, b = y
 * -----------------------------------------------------------------------------
 */
ONESTICK_T_TwoDimFloat ONESTICK_create_dual_channel_dead_spot(ONESTICK_T_TwoDimFloat dd, float zoneIn) {
  bool bandX, bandY;
  ONESTICK_T_TwoDimFloat rr;
  
  bandX = dd.a < -zoneIn || dd.a > zoneIn;  // 0 near zero, else true
  bandY = dd.b < -zoneIn || dd.b > zoneIn;  // 0 near zero, else true
  
  if (bandX) {
    if (bandY) { // region 3
      rr.a = dd.a;
      rr.b = dd.b;
    } else {     // region 2
      rr.a = dd.a;
      rr.b = 0;
    }
  } else {
    if (bandY) { // region 1
      rr.a = 0;
      rr.b = dd.b;
    } else {     // region 0
      rr.a = 0;
      rr.b = 0;
    }
  }
  return rr;
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_low_pass_filter
 * Parameters:    x      = filter input
 *                yPrev  = previous filter output
 *                alpha  = filter coefficient, 0 < alpha < 1.0
 * Return:        y      = filter output
 * Description:
 *   Low pass Infinite Impulse Response (IIR) filter:
 *
 *   y[n] = (1-alpha) * y[n-1] + alpha * x[n]
 *        = y[n-1] + alpha * (x[n] - y[n-1])
 *
 *   When alpha = 1.0 then the filter has no effect, so then y = x.
 * Remark:
 * . Can not use internal static yPrev, because function may be used for more
 *   than one input.
 * -----------------------------------------------------------------------------
 */
float ONESTICK_low_pass_filter(float x, float yPrev, float alpha) {
  return yPrev + alpha * (x - yPrev);
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_switch_states
 * Parameters:
 * - Input
 *   .  sw         = switch
 *   .  maxCount   = maximum count value
 * - Input/output
 *   . *pPrevSw    = previous switch value, initialize it at *pPrevSw = 0
 *   . *pSwCount   = switch count value, initialize it at *pSwCount = 0
 * Return:
 *   . swState     = 0 when sw == 0, *pSwCount when sw == 1
 * Description:
 *   Use RC switch toggling to increment a switch count value, that wraps back
 *   to 1 after swCount has reached maxCount. The swCount is used to identify
 *   maxCount different swState = swCount when sw is 1 and a default
 *   swState = 0 when sw is 0. For example:
 *
 *   maxCount = 5
 *   sw      = 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 etc
 *   swCount = 0 1 1 2 2 3 3 4 4 5 5 1 1 2 2 3 3 4 4 5 etc
 *   swState = 0 1 0 2 0 3 0 4 0 5 0 1 0 2 0 3 0 4 0 5 etc
 *
 * Remark:
 * . The sw must not bounch between 0 and 1 when it is toggled
 * . Can not use internal static prevSw, because function may be used for 
 *   more than one input.
 * -----------------------------------------------------------------------------
 */
uint8_t ONESTICK_switch_states(bool sw, bool *pPrevSw, uint8_t maxCount, uint8_t *pSwCount) {
  uint8_t swState;
  
  if (sw == 0) {  // sw = 0, keep swCount
    swState = 0;
  } else {        // sw = 1
    if (*pPrevSw == 0) {            // Change swCount when switch off/on
      if (*pSwCount >= maxCount) {
        *pSwCount = 1;              // wrap swCount
      } else {
        *pSwCount = *pSwCount + 1;  // increment swCount
      }
    }                               // Keep swCount
    swState = *pSwCount;
  }
  *pPrevSw = sw;  // keep sw as prevSw for next function call
  return swState;
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_set_state
 * Parameters:
 * - Input/output
 *   . *pSwCount = switch count store, will be reset to 0
 *   . swState   = sets the swState
 * Return: swState
 * Description:
 *   Set the switch state that is controlled by ONESTICK_switch_states().
 * -----------------------------------------------------------------------------
 */
uint8_t ONESTICK_set_state(uint8_t swState, uint8_t *pSwCount) {
  *pSwCount = swState;
  return swState;
}


/* -----------------------------------------------------------------------------
 * Function name: ONESTICK_onestick_polar90
 *                ONESTICK_onestick_polar45
 * Parameters:    x      = control pulse width within +-500 us of one stick left-right input
 *                y      = control pulse width within +-500 us of one stick up-down input
 * Return:        L,R    = control pulse width for lefty track L and right track R
 * Description:
 *   Use polar coordinates to determine speed and direction
 *   The speed is controlled by the radius rad and the direction by the angle
 *   phi:
 *     phi =   0  45  90 135 180 225 270 315
 *             0  +1  +1  +1   0  -1  -1  -1  = sin(phi)
 *       L =  +1  +1  +1   0  -1  -1  -1   0  = sin(phi + 45) * rad
 *       R =  -1   0  +1  +1  +1   0  -1  -1  = sin(phi - 45) * rad
 *   For polar90 the speed range is scaled dependent on phi in all quadrants
 *   For polar45 the speed range is scaled dependent on 2*phi in two quadrants
 *   and kept constant in the other two quadrants.
 * Remark:
 * . Run one_stick_caterpillar.py to simulate and view the polar90 and polar45
 *   schemes.
 * -----------------------------------------------------------------------------
 */
ONESTICK_T_TwoDimFloat ONESTICK_onestick_polar90(float x, float y) {
  ONESTICK_T_TwoDimFloat LR;
  float rad, phi, L, R;

  // Polar radius, angle rotated by +45 degrees (is counter clock) to get L, R
  rad = pow(x*x + y*y, 0.5);
  phi = atan2(y, x);
  L = rad * sin(phi + M_PI_4);
  R = rad * sin(phi - M_PI_4);

  // Scale track speed to +-1, by dividing by 1 to sqrt(2) dependent on phi per octant.
  if (ONESTICK_OCTANT_2_3(x, y)) {L *=  sin(phi); R *=  sin(phi);}
  if (ONESTICK_OCTANT_4_5(x, y)) {L *= -cos(phi); R *= -cos(phi);}
  if (ONESTICK_OCTANT_6_7(x, y)) {L *= -sin(phi); R *= -sin(phi);}
  if (ONESTICK_OCTANT_8_1(x, y)) {L *=  cos(phi); R *=  cos(phi);}

  // Return L,R via struct
  LR.a = L;
  LR.b = R;
  return LR;
}


ONESTICK_T_TwoDimFloat ONESTICK_onestick_polar45(float x, float y) {
  ONESTICK_T_TwoDimFloat LR;
  float rad, phi, L, R;

  // Polar radius, angle rotated by +45 degrees (is counter clock) to get L, R
  rad = pow(x*x + y*y, 0.5);
  phi = atan2(y, x);
  L = rad * sin(phi + M_PI_4);
  R = rad * sin(phi - M_PI_4);
           
  // Use cos_triangle() instead of cos() to have avoid zero slope of cos() at edge of transition region
  //if (ONESTICK_QUADRANT_1(x, y)) {L =  rad; R = -rad * ONESTICK_cos_triangle(2*phi);}
  //if (ONESTICK_QUADRANT_2(x, y)) {R =  rad; L = -rad * ONESTICK_cos_triangle(2*phi);}
  //if (ONESTICK_QUADRANT_3(x, y)) {L = -rad; R =  rad * ONESTICK_cos_triangle(2*phi);}
  //if (ONESTICK_QUADRANT_4(x, y)) {R = -rad; L =  rad * ONESTICK_cos_triangle(2*phi);}
  if (ONESTICK_QUADRANT_1(x, y)) {L =  rad; R = -rad * cos(2*phi);}
  if (ONESTICK_QUADRANT_2(x, y)) {R =  rad; L = -rad * cos(2*phi);}
  if (ONESTICK_QUADRANT_3(x, y)) {L = -rad; R =  rad * cos(2*phi);}
  if (ONESTICK_QUADRANT_4(x, y)) {R = -rad; L =  rad * cos(2*phi);}
  
  // Scale track speed to +-1, by dividing by 1 to sqrt(2) dependent on phi per octant.
  if (ONESTICK_OCTANT_1(x, y)) {L *=  cos(phi);}
  if (ONESTICK_OCTANT_2(x, y)) {L *=  sin(phi);}
  if (ONESTICK_OCTANT_3(x, y)) {R *=  sin(phi);}
  if (ONESTICK_OCTANT_4(x, y)) {R *= -cos(phi);}
  if (ONESTICK_OCTANT_5(x, y)) {L *= -cos(phi);}
  if (ONESTICK_OCTANT_6(x, y)) {L *= -sin(phi);}
  if (ONESTICK_OCTANT_7(x, y)) {R *= -sin(phi);}
  if (ONESTICK_OCTANT_8(x, y)) {R *=  cos(phi);}

  // Return L,R via struct
  LR.a = L;
  LR.b = R;
  return LR;
}

/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_onestick_rotate45
 * Parameters:    x      = control pulse width within +-range us of one stick left-right input
 *                y      = control pulse width within +-range us of one stick up-down input
 *                skew   = when skew = 0 then the angle of the track zero speed line is at 45
 *                         degrees for the R track and at -45 degrees for the L track. With skew
 *                         > 0 these lines can be rotated towards 0 degrees to have more
 *                         sensitivity between forward and and cornering with only one track
 *                         running. The resolution between one track running and both tracks
 *                         running in opposite is then reduced correspondingly.
 * Return:        L,R    = control pulse width for lefty track L and right track R
 * Description:
 *   Rotate x = ch1, y = ch2 one stick control by +45 degrees (is counter clock) to get control
 *   for L, R. Using the rotated L and R track speeds directly is not posible, because their
 *   range is -+2 instead of -+1. Therefore the L and R track speeds need to be mapped to the
 *   -+1 range.  The mapping differs per octant, the octants are  numbered as 1 to 8 and the
 *   quadrants as I, II, III, IV:
 *
 *                      y
 *
 *             II     3 | 2      I
 *                  4   |   1
 *             ---------*--------- x
 *                  5   |   8
 *             III    6 | 7     IV
 *
 *   Proportional scaling per octant is used, with or without some skew, to map
 *   the +-2 range to the +-1 range.
 * Remark:
 * . Scheme rotate45 (with skew = 0) and scheme polar45 are equivalent. It 
 *   appears that using cos() and sin() does not take too much time in the main
 *   loop().
 * . Run one_stick_caterpillar.py to simulate and view the rotate45 scheme.
 * -----------------------------------------------------------------------------
 */
ONESTICK_T_TwoDimFloat ONESTICK_onestick_rotate45(float x, float y, float skew) {
  ONESTICK_T_TwoDimFloat LR;
  float L = 0.0;
  float R = 0.0;

  // Rotate XY stick control ch1, ch2 by +45 degrees (is counter clock) to get L, R
  // . scale left track speed in quadrant I and III
  if (ONESTICK_OCTANT_1_5(x, y)) {L =  x; R = y - x + skew*x;}
  if (ONESTICK_OCTANT_2_6(x, y)) {L =  y; R = y - x + skew*x;}
  // . scale right track speed in quadrant II and IV
  if (ONESTICK_OCTANT_3_7(x, y)) {L = y + x - skew*x; R =  y;}
  if (ONESTICK_OCTANT_4_8(x, y)) {L = y + x + skew*y; R = -x;}

  // Return L,R via struct
  LR.a = L;
  LR.b = R;
  return LR;
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_onestick_direction_speed
 * Parameters:    x      = control pulse width within +-500 us of one stick left-right input
 *                y      = control pulse width within +-500 us of one stick up-down input
 *                bidir  = when true then track speeds can become opposite to rotate on the spot,
 *                         else track speeds keep same sign and can at minimum become zero.
 * Return:        L,R    = control pulse width for lefty track L and right track R
 * Description:
 *   Let y = ch2 of one stick control determine the speed and x = ch1 of one stick control
 *   determine the direction. The speed is controlled via y. Relative to this speed the x controls
 *   the direction dependent on bidir to become:
 *   . between same as y, 0 and -y when bidir = true
 *   . between same as y and 0 when bidir = false
 * Remark:
 * . The x,y and L,R pulses are within +-500 us, because the scheme needs to scale for the
 *   maximum pulse width, which is ONESTICK_CH_NORM = 500 us for RC channel PWM control.
 * . Run one_stick_caterpillar.py to simulate and view the direction_speed scheme.
 * -----------------------------------------------------------------------------
 */
ONESTICK_T_TwoDimFloat ONESTICK_onestick_direction_speed(float x, float y, bool bidir) {
  ONESTICK_T_TwoDimFloat LR;
  float L = 0.0;
  float R = 0.0;
  float d;
  
  d = x*y/ONESTICK_CH_NORM;
  if (bidir) {
    d *= 2;
  }

  /* Speed y and direction x stick control */
  if (ONESTICK_QUADRANT_1(x, y)) {L = y;     R = y - d;}
  if (ONESTICK_QUADRANT_2(x, y)) {L = y + d; R = y;}
  if (ONESTICK_QUADRANT_3(x, y)) {L = y + d; R = y;}
  if (ONESTICK_QUADRANT_4(x, y)) {L = y;     R = y - d;}

  // Return L,R via struct
  LR.a = L;
  LR.b = R;
  return LR;
}


/* -----------------------------------------------------------------------------
 * Function name  ONESTICK_onestick_turn_drive
 * Parameters:
 * - Input
 *   x   = control pulse width within +-500 us of one stick left-right input
 *   y   = control pulse width within +-500 us of one stick up-down input
 * - Output
 *   tdMode = returned L,R applies turning or driving
 * Return:
 *   L,R = control pulse width for lefty track L and right track R
 * Description:
 *   Uses only x or only y, dependent on which is the largest. The x is used to run left and
 *   right in opposite clockwise or opposite counter clock wise direction. The y is used to
 *   run both left and right at same speed forward or backward.
 * Remark:
 * . Typically one should not move the stick along or across a diagonal at larger stick
 *   extensions. However by using external low pass filtering of the return values it is
 *   possible to smoothen the large changes in L, R.
 * -----------------------------------------------------------------------------
 */
ONESTICK_T_TwoDimFloat ONESTICK_onestick_turn_drive(float x, float y, ONESTICK_T_mode_turn_or_drive *tdMode) {
  ONESTICK_T_TwoDimFloat LR;

  if (fabs(y) >= fabs(x)) {
	// L, R at same speed given by drive speed y
	*tdMode = ONESTICK_MODE_DRIVE;
    LR.a = y;
    LR.b = y;
  } else {
	// L, R at opposite speed given by turn speed x
	// . x > 0 will turn clockwise and x < 0 will turn counter clockwise, if a = L, b = R
	// . use x to determine turn speed
	*tdMode = ONESTICK_MODE_TURN;
    LR.a = x;
    LR.b = -x;
  } 
  return LR;
}

/* private function implements */
