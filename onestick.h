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

/* -----------------------------------------------------------------------------
 * Purpose:  This library supports receiving and using Radio Control (RC)
 *           channel control in a microcontroller.
 *
 * Functions:
 *
 *   - To check and calibrate received radio control (RC) channels.
 *   - To condition the static and dynamic response of the RC channels.
 *   - To mix the two channel stick control from X,Y to L,R.
 *
 * Acknowledgement:
 *   The ONESTICK_measure_rx_pulse_width() function and how it is used in the PCINT ISR() are
 *   based on similar PCINT ISR() by Joop Brokking in YMFC-3D_V2.
 */

#ifndef ONESTICK_H
#define ONESTICK_H


/* includes */
#include <inttypes.h>
#include <stdbool.h>

/* public constants */

// Use ADC inputs to control the servo channels
const int16_t  ONESTICK_ADC_RANGE = 1024;
const int16_t  ONESTICK_ADC_MAX   = (ONESTICK_ADC_RANGE-1);
const uint16_t ONESTICK_ADC_MID   = (ONESTICK_ADC_RANGE/2);   // to use ADC input as digital input

// Default standard servo channel pulse width for Timer1
const int16_t ONESTICK_CH_WIDTH_MIN = 1000;  // [us], minimum servo pulse width is 540 us
const int16_t ONESTICK_CH_WIDTH_MAX = 2000;  // [us]

// Use ONESTICK_CH_CENTER to translate between servo channel pulse width ONESTICK_CH_CENTER
// -+ONESTICK_CH_NORM range to -+ONESTICK_CH_NORM range
const int16_t ONESTICK_CH_CENTER = ((ONESTICK_CH_WIDTH_MAX + ONESTICK_CH_WIDTH_MIN)/2);  // = 1500 us for standard servo pulse
const int16_t ONESTICK_CH_NORM   = ((ONESTICK_CH_WIDTH_MAX - ONESTICK_CH_WIDTH_MIN)/2);  // =  500 us for standard servo pulse


/* public macros */

#define ONESTICK_MAX(a, b) ((a) > (b) ? (a) : (b))
#define ONESTICK_SEL_A_B(sel, a, b) ((sel) ? (a) : (b))

// Octant conditions, excluding (0, 0). Default initalize result for (x,y) = (0,0), because (0,0)
// does not belong to an octant or quadrant
#define ONESTICK_OCTANT_1(x, y) (x >  0 and y >= 0 and x >   y)  // including positive x-axis
#define ONESTICK_OCTANT_2(x, y) (x >  0 and y >  0 and x <=  y)  // including upper right diagonal
#define ONESTICK_OCTANT_3(x, y) (x <= 0 and y >  0 and y >  -x)  // including positive y-axis
#define ONESTICK_OCTANT_4(x, y) (x <  0 and y >  0 and y <= -x)  // including upper left diagonal
#define ONESTICK_OCTANT_5(x, y) (x <  0 and y <= 0 and x <   y)  // including negative x-axis
#define ONESTICK_OCTANT_6(x, y) (x <  0 and y <  0 and x >=  y)  // including lower left diagonal
#define ONESTICK_OCTANT_7(x, y) (x >= 0 and y <  0 and y <  -x)  // including negative y-axis
#define ONESTICK_OCTANT_8(x, y) (x >  0 and y <  0 and y >= -x)  // including lower right diagonal

#define ONESTICK_OCTANT_1_5(x, y) (ONESTICK_OCTANT_1(x, y) or ONESTICK_OCTANT_5(x, y))  // including x-axis   (y =  0)
#define ONESTICK_OCTANT_2_6(x, y) (ONESTICK_OCTANT_2(x, y) or ONESTICK_OCTANT_6(x, y))  // including diagonal (x =  y)
#define ONESTICK_OCTANT_3_7(x, y) (ONESTICK_OCTANT_3(x, y) or ONESTICK_OCTANT_7(x, y))  // including y-axis   (x =  0)
#define ONESTICK_OCTANT_4_8(x, y) (ONESTICK_OCTANT_4(x, y) or ONESTICK_OCTANT_8(x, y))  // including diagonal (x = -y)

#define ONESTICK_OCTANT_2_3(x, y) ((y > 0) and (x > 0 and x <=  y) or (x <= 0 and y > -x))  // including upper right diagonal
#define ONESTICK_OCTANT_4_5(x, y) ((x < 0) and (y > 0 and y <= -x) or (y <= 0 and x <  y))  // including upper left diagonal
#define ONESTICK_OCTANT_6_7(x, y) ((y < 0) and (x < 0 and x >=  y) or (x >= 0 and y < -x))  // including lower left diagonal
#define ONESTICK_OCTANT_8_1(x, y) ((x > 0) and (y < 0 and y >= -x) or (y >= 0 and x >  y))  // including lower right diagonal

#define ONESTICK_OCTANT_2_3_6_7(x, y) (ONESTICK_OCTANT_2_3(x, y) or ONESTICK_OCTANT_6_7(x, y))  // including x =  y diagonal
#define ONESTICK_OCTANT_8_1_4_5(x, y) (ONESTICK_OCTANT_8_1(x, y) or ONESTICK_OCTANT_4_5(x, y))  // including x = -y diagonal

#define ONESTICK_QUADRANT_1(x, y) (x >  0 and y >= 0)  // including positive x-axis
#define ONESTICK_QUADRANT_2(x, y) (x <= 0 and y >  0)  // including positive y-axis
#define ONESTICK_QUADRANT_3(x, y) (x <  0 and y <= 0)  // including negative x-axis
#define ONESTICK_QUADRANT_4(x, y) (x >= 0 and y <  0)  // including negative y-axis


/* public types */
typedef struct {
  float  a;
  float  b;
} ONESTICK_T_TwoDimFloat;

// Multi ARM control modes
enum ONESTICK_T_mode_turn_or_drive {
  ONESTICK_MODE_TURN      = 0,  // use opposite control to turn direction
  ONESTICK_MODE_DRIVE     = 1   // use same control to drive in one direction
};


/* public variables */

/* public function protos */

/*-----------------------------------------------------------------------------
 * Functions: To create waveforms
 * Description:
 *
 *   - ONESTICK_sin_triangle() create triangle with range +-1 and same phase as sin()
 *   - ONESTICK_cos_triangle() create triangle with range +-1 and same phase as cos()
 *   - ONESTICK_sin_square() create square with range +-1 and same phase as sin()
 *   - ONESTICK_cos_square() create square with range +-1 and same phase as cos()
 */
float ONESTICK_sin_triangle(float phi);
float ONESTICK_cos_triangle(float phi);
float ONESTICK_sin_square(float phi);
float ONESTICK_cos_square(float phi);


/* -----------------------------------------------------------------------------
 * Functions: To measure, check and calibrate received radio control (RC) channels
 * Description:
 *
 *   - ONESTICK_measure_rx_pulse_width() measure received (Rx) pulse width, within an PCINT ISR()
 *   - ONESTICK_check_rx_active() returns active = true when all Radio Control (RC) channels
 *     receive RC, so the user has switched on the RC transmitter and receiver.
 *   - ONESTICK_calibrate_rx_min_max() returns calibrated = true when for all RC channels the
 *     minimum and maximum channel values have been received, so the user has moved the stick
 *     controls to the maximum up and down, or left and right, and then back to neutral center
 *     stick position. For switch RC controls that only have minimum or maximum, so no neutral mid
 *     position, do not need to be calibrated.
 *   - ONESTICK_control_status_led() can show the state of the RC control via an LED (blink,
 *     on, off) by calling it every half blinking period.
 *
 *   Together stable = active && calibrated define stable RC control that can be used to control
 *   an RC device. While the RC control is not stable yet, then the RC device should remain in a
 *   safe default state and ignore any RC control input.
 *
 */
void ONESTICK_measure_rx_pulse_width(uint8_t pinLevel,
                                     uint32_t currentTimeUs,
                                     uint8_t *pPrevPinLevel,
                                     uint32_t *pRxStartUs,
                                     volatile uint16_t *pRxCnt,
                                     volatile int16_t *pRxWidth);
bool ONESTICK_check_rx_active(uint16_t rxCnt[],
                              uint16_t prevRxCnt[],
                              int16_t rxWidth[],
                              uint16_t nofCh);
bool ONESTICK_calibrate_rx_min_max(int16_t chWidth[],
                                   int16_t chMin[],
                                   int16_t chMax[],
                                   uint16_t nofCh);
void ONESTICK_control_status_led(bool active, bool calibrated, bool *ledLight);


/* -----------------------------------------------------------------------------
 * Functions: To condition the static and dynamic response of the RC channels
 * Description:
 *
 *   - ONESTICK_invert_channel_pulse() can be applied to both switch or stick control RC channels.
 *     A switch control RC channel typically only uses the minimum and maximum channel pulse
 *     widths. A stick control RC channel uses the full range of the channel pulse widths and the
 *     cenral position of the stick corresponds to the neutral channel width.
 *
 *   - ONESTICK_remove_channel_pulse_offset(),
 *     ONESTICK_create_exponential_channel(),
 *     ONESTICK_create_dual_slope_channel() from (0,0) via switch point (Xp,Yp) to end (1,1),
 *     ONESTICK_create_channel_dead_zone() change the static shape of channel control, to make
 *       it more or less responsive for small band zone near neutral.
 *     ONESTICK_create_dual_channel_dead_spot() use dual channel spot zone where response is
 *       forced to zero.
 *
 *   - ONESTICK_low_pass_filter() changes the dynamic behavior of the stick control, to make it
 *       more or less responsive in time.
 */
int16_t ONESTICK_invert_channel_pulse(int16_t chWidth, bool invert);    // 1000 us <= chWidth <= 2000 us

int16_t ONESTICK_remove_channel_pulse_offset(int16_t chWidth);          // 1000 us <= chWidth <= 2000 us return -500 us <= y <= 500 us
float ONESTICK_create_exponential_channel(float x, float exponent);         // -500 us <= x <= 500 us
float ONESTICK_create_dual_slope_channel(float x, float Xp, float Yp);      // -500 us <= x <= 500 us
float ONESTICK_create_channel_dead_zone(float x, float zoneX, float zoneY); // -500 us <= x <= 500 us
ONESTICK_T_TwoDimFloat ONESTICK_create_dual_channel_dead_spot(ONESTICK_T_TwoDimFloat dd, float zoneIn); // -500 us <= dd.a, dd.b <= 500 us

float ONESTICK_low_pass_filter(float x, float yPrev, float alpha);


/* -----------------------------------------------------------------------------
 * Functions: To handle channel switch control
 * Description:
 *
 *   - ONESTICK_switch_states() derive a state from toggling a switch. When
 *     switch is off then state is 0, else when switch becomes on then state
 *     increments.
 *   - ONESTICK_reset_state() to reset the state that is maintained by
 *     ONESTICK_switch_states(). 
 */
uint8_t ONESTICK_switch_states(bool sw, bool *pPrevSw, uint8_t maxCount, uint8_t *pSwCount);

uint8_t ONESTICK_set_state(uint8_t swState, uint8_t *pSwCount);


/* -----------------------------------------------------------------------------
 * Functions: To mix the two channel stick control from X,Y to L,R
 * Description:
 *
 *   - ONESTICK_onestick_polar90(),
 *     ONESTICK_onestick_polar45(),
 *     ONESTICK_onestick_rotate45() rotate X and Y stick control by 45 degrees to have L and R
 *     for left and right caterpillar track control. Both schemes are similar from a user point of
 *     view, but differ in implementation. The polar90 scheme uses sin() and cos() whereas
 *     rotate45 scheme uses linearized calculations.
 *
 *   - ONESTICK_onestick_direction_speed() uses X for direction and Y for speed. With bidir = true 
 *     the speed of the other track can become opposite to rotate on the spot. With bidir = false
 *     the speed of the other track can become zero, but cannot change sign, so both tracks keep
 *     same direction.
 *
 *   - ONESTICK_onestick_turn_drive() uses only X or only Y, dependent on which is the largest.
 *     The X is used to run left and right in opposite clockwise or counter clock wise direction.
 *     The Y is used to run both left and right at same speed forward or backward.
 *
 *   In general the polar90 scheme is preferred, because it is more smooth and sufficently fast
 *   compared to the rotate45 scheme, and more intuitive than the direction_speed scheme.
 */
ONESTICK_T_TwoDimFloat ONESTICK_onestick_polar90(float x, float y);
ONESTICK_T_TwoDimFloat ONESTICK_onestick_polar45(float x, float y);
ONESTICK_T_TwoDimFloat ONESTICK_onestick_rotate45(float x, float y, float skew);
ONESTICK_T_TwoDimFloat ONESTICK_onestick_direction_speed(float x, float y, bool bidir);
ONESTICK_T_TwoDimFloat ONESTICK_onestick_turn_drive(float x, float y, ONESTICK_T_mode_turn_or_drive *tdMode);

#endif

