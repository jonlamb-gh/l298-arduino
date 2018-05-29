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
#define PWM_PERIOD (200000UL)
#define PWM_PERIOD_MIN (100UL)
#define PWM_PERIOD_MAX (500000UL)

// 10 - 6.56 Hz
//#define PWM_PERIOD_MIN (100000UL)
//#define PWM_PERIOD_MAX (180000UL)

// 6.94 - 8 Hz
//#define PWM_PERIOD_MIN (144000UL)
//#define PWM_PERIOD_MAX (125000UL)

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
} input_s;

typedef struct
{
    uint16_t ena;
    uint8_t in1;
    uint8_t in2;
    uint32_t timer_period;
} output_s;

static volatile uint8_t timer_state;

static void timer_callback(void)
{
    digitalWrite(PIN_LED, timer_state);

    timer_state = !timer_state;
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
}

static void write_output(
        const output_s * const output)
{
    Timer3.setPeriod(output->timer_period);
    digitalWrite(PIN_IN1, (output->in1 == 0 ? LOW : HIGH));
    digitalWrite(PIN_IN2, (output->in2 == 0 ? LOW : HIGH));
    analogWrite(PIN_ENA, output->ena);
}

static void print_input(
        const input_s * const input)
{
    Serial.print("BT0: ");
    Serial.print(input->btn0);
    Serial.print(" BT1: ");
    Serial.print(input->btn1);
    Serial.print(" BT2: ");
    Serial.print(input->btn2);
    Serial.print(" PT0: ");
    Serial.print(input->pot0);
    Serial.print(" PT1: ");
    Serial.print(input->pot1);
    Serial.print(" CSA: ");
    Serial.println(input->csa);
}

static void print_output(
        const output_s * const output)
{
    const float freq = 1000000.0f / (float) output->timer_period;

    Serial.print("IN1: ");
    Serial.print(output->in1);
    Serial.print(" IN2: ");
    Serial.print(output->in2);
    Serial.print(" ENA: ");
    Serial.print(output->ena);
    Serial.print(" T3P: ");
    Serial.print(output->timer_period);
    Serial.print(" T3F: ");
    Serial.println(freq);
}

static void disable_output(void)
{
    digitalWrite(PIN_ENA, LOW);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
}

void setup(void)
{
    timer_state = 0;

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_CSA, INPUT);

    pinMode(PIN_BTN0, INPUT_PULLUP);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);
    pinMode(PIN_POT0, INPUT);
    pinMode(PIN_POT1, INPUT);

    disable_output();

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    Timer3.initialize(PWM_PERIOD);
    Timer3.attachInterrupt(&timer_callback);

    Serial.begin(115200);
}

void loop(void)
{
    // pot0 -> pwm frequency (should be fixed?)
    // pot1 -> direction toggle frequency
    //
    // bt0 ->
    // bt1 ->
    // bt2 ->

    input_s input;
    output_s output;

    disable_output();

    Serial.println("\n\nwaiting for btn0 press\n");

    // wait for btn0 to be pressed
    do
    {
        read_input(&input);
    }
    while(input.btn0 == 0);

    Serial.println("\nloop until btn1 press\n");
    delay(50);

    // wait for btn1 to be pressed
    do
    {
        read_input(&input);
        print_input(&input);

        delay(50);
    }
    while(input.btn1 == 0);

    Serial.println("\nenabling output until btn2 press\n");
    delay(50);

    // drive output loops until btn2 is pressed
    do
    {
        read_input(&input);

        if(input.btn1 != 0)
        {
            if(timer_state == 0)
            {
                output.in1 = 1;
                output.in2 = 0;
            }
            else
            {
                output.in1 = 0;
                output.in2 = 1;
            }

            output.ena = map(
                    input.pot0,
                    0,
                    1023,
                    0,
                    255);

            output.timer_period = map(
                    input.pot1,
                    0,
                    1023,
                    PWM_PERIOD_MIN,
                    PWM_PERIOD_MAX);

            write_output(&output);
        }
        else
        {
            disable_output();
        }

        print_input(&input);
        print_output(&output);

        delay(20);
    }
    while(input.btn2 == 0);

    disable_output();
    delay(1000);
}
