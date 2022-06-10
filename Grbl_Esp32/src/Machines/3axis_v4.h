#pragma once
// clang-format off

/*
    3axis_v4.h
    Part of Grbl_ESP32

    Pin assignments for the ESP32 Development Controller, v4.1 and later.
    https://github.com/bdring/Grbl_ESP32_Development_Controller
    https://www.tindie.com/products/33366583/grbl_esp32-cnc-development-board-v35/

    2018    - Bart Dring
    2020    - Mitch Bradley

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/

#define MACHINE_NAME            "ESP32_V4"

#define X_STEP_PIN              GPIO_NUM_19
#define X_DIRECTION_PIN         GPIO_NUM_18
#define Y_STEP_PIN              GPIO_NUM_16
#define Y_DIRECTION_PIN         GPIO_NUM_4

#define X_LIMIT_PIN             GPIO_NUM_26
#define Y_LIMIT_PIN             GPIO_NUM_33


// OK to comment out to use pin for other features
#define STEPPERS_DISABLE_PIN    GPIO_NUM_17

#define SPINDLE_TYPE            SpindleType::NONE

#define DEFAULT_X_STEPS_PER_MM 509.2958
#define DEFAULT_Y_STEPS_PER_MM 509.2958

#define DEFAULT_X_MAX_RATE 12000.0   // mm/min
#define DEFAULT_Y_MAX_RATE 12000.0   // mm/min

#define DEFAULT_X_ACCELERATION 50.0 // mm/sec^2. 500 mm/sec^2 = 1800000 mm/min^2
#define DEFAULT_Y_ACCELERATION 50.0 // mm/sec^2

/*
#define CONTROL_SAFETY_DOOR_PIN GPIO_NUM_35  // labeled Door,  needs external pullup
#define CONTROL_RESET_PIN       GPIO_NUM_34  // labeled Reset, needs external pullup
#define CONTROL_FEED_HOLD_PIN   GPIO_NUM_36  // labeled Hold,  needs external pullup
#define CONTROL_CYCLE_START_PIN GPIO_NUM_39  // labeled Start, needs external pullup
*/

