/*
    SpindleClass.h

    Header file for a Spindle Class
    See SpindleClass.cpp for more details

    Part of Grbl_ESP32

    2020 -	Bart Dring This file was modified for use on the ESP32
                    CPU. Do not use this with Grbl for atMega328P

    Grbl is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#define SPINDLE_STATE_DISABLE  0  // Must be zero.
#define SPINDLE_STATE_CW       bit(0)
#define SPINDLE_STATE_CCW      bit(1)


#define SPINDLE_TYPE_NONE       0
#define SPINDLE_TYPE_PWM        1
#define SPINDLE_TYPE_RELAY      2
#define SPINDLE_TYPE_LASER      3
#define SPINDLE_TYPE_DAC        4
#define SPINDLE_TYPE_HUANYANG   5
#define SPINDLE_TYPE_BESC       6

#ifndef SPINDLE_CLASS_H
#define SPINDLE_CLASS_H

#include "../grbl.h"
#include <driver/dac.h>
#include "driver/uart.h"



// This is the base class. Do not use this as your spindle
class Spindle {
  public:
    virtual void init(); // not in constructor because this also gets called when $$ settings change
    virtual float set_rpm(float rpm);
    virtual void set_state(uint8_t state, float rpm);
    virtual uint8_t get_state();
    virtual void stop();
    virtual void config_message();
    virtual bool isRateAdjusted();
    virtual void spindle_sync(uint8_t state, float rpm);

    bool is_reversable; 
};

// This is a dummy spindle that has no I/O.
// It is used to ignore spindle commands when no spinde is desired
class NullSpindle : public Spindle {
  public:
    void init();
    float set_rpm(float rpm);
    void set_state(uint8_t state, float rpm);
    uint8_t get_state();
    void stop();
    void config_message();
};

// This adds support for PWM
class PWMSpindle : public Spindle {
  public:
    void init();
    virtual float set_rpm(float rpm);
    void set_state(uint8_t state, float rpm);
    uint8_t get_state();
    void stop();
    void config_message();

  private:

    int32_t _current_pwm_duty;
    void set_spindle_dir_pin(bool Clockwise);

  protected:
    float _min_rpm;
    float _max_rpm;
    uint32_t _pwm_off_value;
    uint32_t _pwm_min_value;
    uint32_t _pwm_max_value;
    uint8_t _output_pin;
    uint8_t _enable_pin;
    uint8_t _direction_pin;
    int8_t _spindle_pwm_chan_num;
    float _pwm_freq;
    uint32_t _pwm_period; // how many counts in 1 period
    uint8_t _pwm_precision;
    float _pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.

    virtual void set_output(uint32_t duty);
    void set_enable_pin(bool enable_pin);
    void get_pins_and_settings();
    uint8_t calc_pwm_precision(float freq);
};

// This is for an on/off spindle all RPMs above 0 are on
class RelaySpindle : public PWMSpindle {
  public:
    void init();
    void config_message();
    float set_rpm(float rpm);
  protected:
    void set_output(uint32_t duty);
};

// this is the same as a PWM spindle, but the M4 compensation is supported.
class Laser : public PWMSpindle {
  public:
    bool isRateAdjusted();
    void config_message();
};

// This uses one of the (2) DAC pins on ESP32 to output a voltage
class DacSpindle : public PWMSpindle {
  public:
    void init();
    void config_message();
    float set_rpm(float rpm);
  private:
    bool _gpio_ok; // DAC is on a valid pin
  protected:
    void set_output(uint32_t duty); // sets DAC instead of PWM
};

class HuanyangSpindle : public Spindle {
  public:
    void init();
    void config_message();
    virtual void set_state(uint8_t state, float rpm);
    uint8_t get_state();
    float set_rpm(float rpm);
    void stop();

  private:
    bool get_response(uint16_t length);
    uint16_t  ModRTU_CRC(char* buf, int len);
    void add_ModRTU_CRC(char* buf, int full_msg_len);
    bool set_mode(uint8_t mode);
    bool get_pins_and_settings();

    uint8_t _txd_pin;
    uint8_t _rxd_pin;
    uint8_t _rts_pin;
    uint8_t _state;
};

class BESCSpindle : public PWMSpindle {
  public:
    void init();
    void config_message();
    float set_rpm(float rpm);
};

extern Spindle* spindle;

extern NullSpindle null_spindle;
extern PWMSpindle pwm_spindle;
extern RelaySpindle relay_spindle;
extern Laser laser;
extern DacSpindle dac_spindle;
extern HuanyangSpindle huanyang_spindle;
extern BESCSpindle besc_spindle;

void spindle_select(uint8_t spindle_type);
void spindle_read_prefs(Preferences& prefs);

#endif
