/**
 * @file l298_ard.ino
 * @brief TODO.
 *
 * Board: Mega ADK
 * Version: 1.8.5
 *
 */

#include <stdint.h>
#include <TimerThree.h>

// frequency = 1,000,000 / period
// period = 1,000,000 / frequency

//#define PWM_PERIOD_MIN (5UL)
//#define PWM_PERIOD_MAX (500UL)

#define PWM_PERIOD_MIN (5UL)
#define PWM_PERIOD_MAX (800UL)

#define PIN_BTN0 (21)
#define PIN_BTN1 (20)
#define PIN_BTN2 (18)
#define PIN_POT0 (A0)
#define PIN_POT1 (A1)

#define PIN_ENA (7)
#define PIN_IN1 (9)
#define PIN_IN2 (8)
#define PIN_CSA (A2)
#define PIN_LED (13)

typedef struct
{
    uint8_t btn0;
    uint8_t btn1;
    uint8_t btn2;
    uint16_t pot0;
    uint16_t pot1;
    uint16_t csa;
    uint16_t map_top;
    uint32_t map_period;
} input_s;

typedef struct
{
    uint8_t dir;
    uint8_t mag;
    uint8_t top;
} waveform_s;

static volatile waveform_s g_waveform;

static void timer_callback(void)
{
    g_waveform.mag += 1;

    if(g_waveform.mag == g_waveform.top)
    {
        g_waveform.mag = 0;
        g_waveform.dir = !g_waveform.dir;
    }

    write_output();
}

static void write_output(void)
{
    if(g_waveform.dir == 0)
    {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_LED, LOW);
    }
    else
    {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_LED, HIGH);
    }

    analogWrite(PIN_ENA, g_waveform.mag);
}

static void set_waveform(
        const uint8_t top,
        const uint8_t dir)
{
    noInterrupts();

    g_waveform.mag = 0;
    g_waveform.top = top;
    g_waveform.dir = dir;

    interrupts();
}

static void read_input(
        input_s * const input)
{
    input->pot0 = (uint16_t) analogRead(PIN_POT0);

    input->btn0 = (uint8_t) !digitalRead(PIN_BTN0);
    input->btn1 = (uint8_t) !digitalRead(PIN_BTN1);
    input->btn2 = (uint8_t) !digitalRead(PIN_BTN2);

    input->pot1 = (uint16_t) analogRead(PIN_POT1);
    input->csa = (uint16_t) analogRead(PIN_CSA);

    input->map_top = map(
            input->pot0,
            0,
            1023,
            1,
            255);

    input->map_period = map(
            input->pot1,
            0,
            1023,
            PWM_PERIOD_MIN,
            PWM_PERIOD_MAX);
}

static void print_input(
        const input_s * const input)
{
    const float freq = 1000000.0f / (float) input->map_period;

    Serial.print(" PT0: ");
    Serial.print(input->pot0);
    Serial.print(" PT1: ");
    Serial.print(input->pot1);
    Serial.print(" CSA: ");
    Serial.print(input->csa);
    Serial.print(" T: ");
    Serial.print(input->map_top);
    Serial.print(" P: ");
    Serial.print(input->map_period);
    Serial.print(" F: ");
    Serial.println(freq);
}

static void disable_output(void)
{
    Timer3.stop();
    digitalWrite(PIN_ENA, LOW);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
}

void setup(void)
{
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_CSA, INPUT);

    pinMode(PIN_BTN0, INPUT_PULLUP);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);
    pinMode(PIN_POT0, INPUT);
    pinMode(PIN_POT1, INPUT);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    disable_output();

    set_waveform(0xFF, 0);

    Timer3.initialize(PWM_PERIOD_MAX);
    Timer3.attachInterrupt(&timer_callback);
    disable_output();

    Serial.begin(115200);
}

void loop(void)
{
    input_s input;

    read_input(&input);

    print_input(&input);

    if(input.btn0 != 0)
    {
        set_waveform(
                input.map_top,
                g_waveform.dir);

        Timer3.setPeriod(input.map_period);

        do
        {
            const uint16_t last_p = input.pot1;

            read_input(&input);

            if(last_p != input.pot1)
            {
                Timer3.setPeriod(input.map_period);
            }
        }
        while(input.btn0 != 0);
    }

    disable_output();
    delay(50);

    //Serial.println("\n\nwaiting for btn0 press\n");
    //disable_output();
    //delay(1000);
}
