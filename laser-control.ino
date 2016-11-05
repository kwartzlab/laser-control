/*
 *  kwartzlab laser control
 *  2016-11 James Bastow <james@jamesbastow.com>
 *  
 *  ARDBOX IS.20AN.base http://www.industrialshields.com/shop/plc-arduino-ardbox-20-ios-analog/
 *  (Arduino Leonardo)
 */

/* 
 * ARDBOX to Arduino pin map
 */
 
#define I9     A0
#define I8     A1
#define I7     A2
#define I6     A3
#define I5     A4
#define I4     A5
#define I3     4
#define I2     8
#define I1     12
#define I0     2
#define Q9     0
#define Q8     1
#define Q7     7
#define Q6     3
#define Q5     5
#define Q4     6
#define Q3     9
#define Q2     10
#define Q1     11
#define Q0     13

/*
 * Laser control configuration
 */
// IO assignments
#define BUTTON_STOP_PIN         I0
#define BUTTON_START_PIN        I1
#define INDICATOR_R_PIN         Q1
#define INDICATOR_G_PIN         Q0
#define INDICATOR_Y_PIN         Q2

#define RELAY_MAIN_PIN          Q4
#define RELAY_VENTILATION_PIN   Q5
#define RELAY_AIR_PIN           Q6
#define RELAY_CHILLER_PIN       Q7
#define RELAY_LASER_PIN         Q8

// IO macros
#define INDICATOR_R_ON()        digitalWrite(INDICATOR_R_PIN, HIGH)
#define INDICATOR_R_OFF()       digitalWrite(INDICATOR_R_PIN, LOW)
#define INDICATOR_G_ON()        digitalWrite(INDICATOR_G_PIN, HIGH)
#define INDICATOR_G_OFF()       digitalWrite(INDICATOR_G_PIN, LOW)
#define INDICATOR_Y_ON()        digitalWrite(INDICATOR_Y_PIN, HIGH)
#define INDICATOR_Y_OFF()       digitalWrite(INDICATOR_Y_PIN, LOW)
#define RELAY_MAIN_ON()         digitalWrite(RELAY_MAIN_PIN, HIGH)
#define RELAY_MAIN_OFF()        digitalWrite(RELAY_MAIN_PIN, LOW)
#define RELAY_VENTILATION_ON()  digitalWrite(RELAY_VENTILATION_PIN, HIGH)
#define RELAY_VENTILATION_OFF() digitalWrite(RELAY_VENTILATION_PIN, LOW)
#define RELAY_AIR_ON()          digitalWrite(RELAY_AIR_PIN, HIGH)
#define RELAY_AIR_OFF()         digitalWrite(RELAY_AIR_PIN, LOW)
#define RELAY_CHILLER_ON()      digitalWrite(RELAY_CHILLER_PIN, HIGH)
#define RELAY_CHILLER_OFF()     digitalWrite(RELAY_CHILLER_PIN, LOW)
#define RELAY_LASER_ON()        digitalWrite(RELAY_LASER_PIN, HIGH)
#define RELAY_LASER_OFF()       digitalWrite(RELAY_LASER_PIN, LOW)

// Cooldown time (minutes)
#define COOLDOWN_TIME           5

// Restart lockout time (minutes)
#define RESTART_LOCKOUT_TIME    1

// Tick period (milliseconds)
#define TICK_RATE               10

// Button debounce time (milliseconds)
#define BUTTON_DEBOUNCE_TIME    10

// Cooldown time (milliseconds)
#define STATE_COOLDOWN_TIME     (5 * 60 * 1000)

// Startup / shutdown power sequence delay (milliseconds)
#define SEQUENCE_DELAY_TIME     500

/*
 * Type definitions
 */
typedef enum {
    STATE_INIT = 0,
    STATE_OFF,
    STATE_ON,
    STATE_COOLDOWN
} states_e;

typedef enum button_state_t {
    BUTTON_STATE_UNKNOWN = 0,
    BUTTON_STATE_DOWN,
    BUTTON_STATE_UP,
};

typedef enum button_event_t {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESSED,
    BUTTON_EVENT_RELEASED,
    BUTTON_EVENT_HELD
};

typedef struct button_t {
    int pin;
    uint8_t _debounce;
    unsigned long _prev_millis;
    unsigned long _down_time;
    
    button_state_t state;
    button_event_t event;
};

/*
 * Functions
 */
button_event_t button_get_event(button_t *b);
button_state_t button_get_state(button_t *b);
void           button_handle(button_t *b);
void           button_init(button_t *b, int pin);
states_e       system_get_state();
void           system_set_state(states_e state);

/*
 * Global variables
 */
button_t button_start;
button_t button_stop;
states_e system_state;

/*
 * Program code
 */
void setup()
{
    Serial.begin(9600);
    //while (!Serial); // This hangs Leonardo board if USB CDC is not available
    
    system_set_state(STATE_INIT);

    pinMode(INDICATOR_R_PIN, OUTPUT);
    pinMode(INDICATOR_G_PIN, OUTPUT);
    pinMode(INDICATOR_Y_PIN, OUTPUT);
    INDICATOR_R_OFF();
    INDICATOR_G_OFF();
    INDICATOR_Y_OFF();

    pinMode(RELAY_MAIN_PIN, OUTPUT);
    pinMode(RELAY_VENTILATION_PIN, OUTPUT);
    pinMode(RELAY_AIR_PIN, OUTPUT);
    pinMode(RELAY_CHILLER_PIN, OUTPUT);
    pinMode(RELAY_LASER_PIN, OUTPUT);
    RELAY_MAIN_OFF();
    RELAY_VENTILATION_OFF();
    RELAY_AIR_OFF();
    RELAY_CHILLER_OFF();
    RELAY_LASER_OFF();
    
    button_init(&button_start, BUTTON_START_PIN);
    button_init(&button_stop, BUTTON_STOP_PIN);
    
    system_set_state(STATE_OFF);
}

void loop()
{
    uint32_t heartbeat = 0;
    uint8_t y = 0;
    unsigned long millis_cur = 0;
    unsigned long millis_prev = 0;
    unsigned long millis_delta = 0;
    unsigned long cooldown_time = 0;
    
    for (;;) {
        switch (system_get_state()) {
            case STATE_INIT:
                //Should not be here
                break;
          
            case STATE_OFF:
                INDICATOR_R_ON();
                INDICATOR_G_OFF();

                if (button_get_event(&button_start) == BUTTON_EVENT_PRESSED) {
                    // Turn on everything
                    INDICATOR_G_ON();
                    INDICATOR_R_OFF();
                    
                    RELAY_MAIN_ON();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_LASER_ON();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_CHILLER_ON();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_VENTILATION_ON();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_AIR_ON();
                    
                    system_set_state(STATE_ON);
                }

                break;
          
            case STATE_ON:
                INDICATOR_R_OFF();
                INDICATOR_G_ON();

                if (button_get_event(&button_stop) == BUTTON_EVENT_PRESSED) {
                    INDICATOR_R_ON();
                    INDICATOR_G_OFF();

                    // Turn off laser & air
                    RELAY_LASER_OFF();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_AIR_OFF();
                    
                    cooldown_time = STATE_COOLDOWN_TIME;
                    system_set_state(STATE_COOLDOWN);
                }
                break;
    
            case STATE_COOLDOWN:
                INDICATOR_G_OFF();
                
                millis_cur = millis();

                // Hold stop for 5s to stop cooldown
                if (button_get_event(&button_stop) == BUTTON_EVENT_HELD) {
                    cooldown_time = 0;
                }
                
                if (cooldown_time == 0) { // Cooldown finished
                   INDICATOR_R_ON();
                   
                    // Turn off everything
                    RELAY_LASER_OFF();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_CHILLER_OFF();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_VENTILATION_OFF();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_AIR_OFF();
                    delay(SEQUENCE_DELAY_TIME);
                    RELAY_MAIN_OFF();
                    
                    system_set_state(STATE_OFF);
                } else { // Cooldown countdown
                    // Calculate elapsed time
                    millis_delta = (unsigned long)(millis_cur - millis_prev);
                    millis_prev = millis_cur;
                    
                    // Count down, avoiding underflow
                    if (cooldown_time > millis_delta) {
                        cooldown_time -= millis_delta;
                    } else {
                        cooldown_time = 0;
                    }

                    // Flash red indicator
                    if ((cooldown_time / 500) % 2) {
                        INDICATOR_R_ON();
                    } else {
                        INDICATOR_R_OFF();
                    }
                }
                break;
    
            default:
                break;
            
        }
        
        button_handle(&button_start);
        button_handle(&button_stop);

        // Heartbeat
        if ((millis() / 1000) % 2) {
            INDICATOR_Y_ON();
        } else {
            INDICATOR_Y_OFF();
        }

        delay(TICK_RATE);
    }    
}

void button_handle(button_t *b)
{
    unsigned long cur_millis = millis();
    unsigned long delta_millis = cur_millis - b->_prev_millis;
    
    if (delta_millis > BUTTON_DEBOUNCE_TIME) {
        b->_debounce <<= 1;
        b->_debounce |= (digitalRead(b->pin) == HIGH) ? 1 : 0;
        b->_prev_millis = cur_millis;
    } else {
        return;
    }

    if (b->_debounce == 0x0F) {
        b->event = BUTTON_EVENT_PRESSED;
        b->state = BUTTON_STATE_DOWN;
        Serial.print("PRESSED\n");
    } else if (b->_debounce == 0xF0) {
        b->event = BUTTON_EVENT_RELEASED;
        b->state = BUTTON_STATE_UP;
        Serial.print("RELEASED\n");
    } else if (b->_debounce == 0xFF) {
        b->state = BUTTON_STATE_DOWN;
        b->_down_time += delta_millis;
    } else if (b->_debounce == 0x00) {
        b->state = BUTTON_STATE_UP;
        b->_down_time = 0;
    }

    // if DOWN for > 5s, button is held
    if (b->_down_time > 5000) {
        b->event = BUTTON_EVENT_HELD;
        b->_down_time = 0;
        Serial.print("HELD\n");
    }
}

button_state_t button_get_state(button_t *b)
{
    return b->state;
}

button_event_t button_get_event(button_t *b)
{
    button_event_t event = b->event;
    b->event = BUTTON_EVENT_NONE;
    return event;
}

void button_init(button_t *b, int pin)
{
    pinMode(pin, INPUT);
    b->pin = pin;
    b->_debounce = 0;
    b->_prev_millis = 0;
    b->_down_time = 0;
    b->event = BUTTON_EVENT_NONE;
    b->state = BUTTON_STATE_UNKNOWN;
}

void system_set_state(states_e state)
{
    system_state = state;
    switch (system_state) {
        case STATE_INIT: Serial.println("STATE_INIT"); break;
        case STATE_ON: Serial.println("STATE_ON"); break;
        case STATE_OFF: Serial.println("STATE_OFF"); break;
        case STATE_COOLDOWN: Serial.println("STATE_COOLDOWN"); break;
    }
}

states_e system_get_state()
{
    return system_state;
}

