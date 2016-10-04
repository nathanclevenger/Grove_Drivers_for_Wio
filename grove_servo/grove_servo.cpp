/*
 * grove_servo.cpp
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Jack Shao (jacky.shaoxg@gmail.com)
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "suli2.h"
#include "grove_servo.h"


GroveServo::GroveServo(int pin)
{
    this->io = (PWM_T *)malloc(sizeof(PWM_T));
    this->timer = (TIMER_T *)malloc(sizeof(TIMER_T));

    suli_pwm_init(this->io, pin);
    suli_pwm_frequency(this->io, 50);
    last_degree = 0;

    percent_start = 0;
    percent_end = 0;
    percent_increment = 0;
    percent_current = 0;
}



bool GroveServo::write_angle(int degree)
{
    last_degree = degree;
    degree = constrain(degree, 0, 180);
    float percent = 10 * degree / 180 + 2.5;
    suli_pwm_frequency(this->io, 50);
    suli_pwm_output(this->io, percent);
    return true;
}

bool GroveServo::write_angle_motion_in_seconds(int degree, int seconds)
{
    last_degree = degree;
    degree = constrain(degree, 0, 180);
    float percent = 10 * degree / 180 + 2.5;
    suli_pwm_frequency(this->io, 50);
    suli_pwm_output(this->io, percent);

    suli_timer_install(timer, seconds * 1000000, grove_servo_timer_interrupt_handler, this, false);

    return true;
}

bool GroveServo::write_move_over_time(int degree_start, int degree_end, int seconds)
{
    last_degree = degree;
    degree = constrain(degree, 0, 180);
    percent_start = 10 * degree_start / 180 + 2.5;
    percent_end = 10 * degree_end / 180 + 2.5;
    percent_increment = (percent_end - percent_start) / (seconds / 50);
    suli_pwm_frequency(this->io, 50);
    suli_pwm_output(this->io, percent);

    suli_timer_install(timer, 20000, grove_servo_move_timer_interrupt_handler, this, true);

    return true;
}

bool GroveServo::read_angle(int *degree)
{
    *degree = last_degree;
    return true;
}

static void grove_servo_timer_interrupt_handler(void *para)
{
    GroveServo *g = (GroveServo *)para;
    suli_pwm_output(g->io, 0);
}

static void grove_servo_move_timer_interrupt_handler(void *para)
{
    GroveServo *g = (GroveServo *)para;

    g->percent_current = g->percent_current + g->percent_increment;

    suli_pwm_frequency(this->io, 50);
    suli_pwm_output(this->io, g->percent_current);

    if (g->percent_increment > 0 && g->percent_current >= percent_end) {
      suli_timer_remove(g->timer);
    } else if (g->percent_increment < 0 && g->percent_current <= percent_end) {
      suli_timer_remove(g->timer);
    }

}
