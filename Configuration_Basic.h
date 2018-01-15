/*
Loonar Technologies User Configuration File

CONFIGURE THIS FLIGHT BEFORE FLIGHT!!!!!!!!!!
 
Copyright 2018 Loonar Technologies, LLC

      THIS SOFTWARE IS PRESENTED WITH THE MIT LICENCE:

* Permission is hereby granted, free of charge, to any person 
* obtaining a copy of this software and associated documentation 
* files (the "Software"), to deal in the Software without 
* restriction, including without limitation the rights to use, 
* copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the 
* Software is furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included 
* in all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR 
* ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef UserConfiguration_h
#define UserConfiguration_h


/********** LIBRARIES **********/
// None at the moment.



/********** PINOUT **********/
const int BREAKOUT_1 =                 3;    // Breakout pin #1 - Labeled as 'GPIO1' on the Mainboard. Digital input/output only 
const int BREAKOUT_2 =                 4;    // Breakout pin #2 - Labeled as 'GPIO2' on the Mainboard. Digital input/output only.
const int BREAKOUT_3 =                 16;   // Breakout pin #3 - Labeled as 'GPIO3/ADC1' on the Mainboard. Digital input/output or analog input only. 
const int BREAKOUT_4 =                 A12;  // Breakout pin #4 - Labeled as 'GPIO3/ADC2/DAC' on the Mainboard. Digital input/output or analog input or DAC output only. 



/********** USER DEFINED CONSTANTS **********/
const double   FREQ =                  145.0;                        // Center frequency in MHz for the radio module.
const uint8_t  FCCID[6] =              {'K','K','6','M','I','S'};    // FCC Radio License Callsign.
const float    LAUNCH_LATITUDE =       0.0000;                       // Launch Location Latitude
const float    LAUNCH_LONGITUDE =      0.0000;                       // Launch Location Longitude

#endif /* UserConfiguration_h */
