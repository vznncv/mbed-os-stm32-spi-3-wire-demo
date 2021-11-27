#ifndef APP_PULSE_COUNTER_H
#define APP_PULSE_COUNTER_H

#include "mbed.h"


/**
 * Pulse counter.
 *
 * Unlike common interrupt base implementation it doesn't consume CPU time and
 * is able to work with higher frequencies.
 */
class PulseCounter : NonCopyable<PulseCounter> {
protected:
    PinName _pin;
    PWMName _pwm;

    static PWMName _get_timer(PinName pin);

    static void _enable_tim_clock(PWMName pwm);

    static int _get_encoder_function(PinName pin, PinMode mode);


public:
    enum PulseEdge {
        RisingEdge = 0,
        FallingEdge = 1
    };

    /**
     * Constructor.
     *
     * @param pin PulseCounter pin to connect to
     * @param edge signal edge to count pulse
     */
    PulseCounter(PinName pin, PulseEdge edge = FallingEdge, PinMode mode = PullNone);

    ~PulseCounter();

    void reset();

    int get_count();

};

#endif // APP_PULSE_COUNTER_H
