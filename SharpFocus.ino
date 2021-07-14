// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       SharpFocus.ino
    Created:	27-05-2021 04:02:25 PM
    Author:     Mandar Potdar
*/

// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//


// Define Functions below here or use other .ino or cpp files
//

#define DEBUG_PRINTS 0

/* List of commands supported
* OUT#<value>
* IN#<value>
* SET_POS#<value>
* GET_POS#
* SET_ZERO_POS#
* SET_MAX_POS#
* SET_MAX_POS_VALUE#<value>
* GET_MAX_POS_VALUE#
* HALT#
*/

// Pin connections to DRV8825 and manual push buttons
static constexpr uint8_t PUSH_IN    = 2;    // [D2][INPUT_PULLUP] Manual push button IN
static constexpr uint8_t PUSH_OUT   = 3;    // [D3][INPUT_PULLUP] Manual push button OUT
static constexpr uint8_t FAULT_     = 4;    // [D4] DRV8825 fault output
static constexpr uint8_t DIR        = 5;    // [D5] Direction of motor
static constexpr uint8_t STEP       = 6;    // [D6] Step DRV8825 on low->high transition
static constexpr uint8_t M2         = 7;    // [D7]
static constexpr uint8_t M1         = 8;    // [D8] DRV8825 MODE pins
static constexpr uint8_t M0         = 9;    // [D9]
static constexpr uint8_t SLP_       = 13;   // [D13] Sleep DRV8825 when LOW
static constexpr uint8_t RST_       = SLP_; // [D13] Reset DRV8825 when LOW

// Direction of focuser
static constexpr uint8_t DIR_IN  = 0;
static constexpr uint8_t DIR_OUT = 1;

// MODE definition                      M2 M1 M0
static constexpr uint8_t MODE_FULL = 0x0; // 0  0  0
static constexpr uint8_t MODE_HALF = 0x1; // 0  0  1
static constexpr uint8_t MODE_4TH  = 0x2; // 0  1  0
static constexpr uint8_t MODE_8TH  = 0x3; // 0  1  1
static constexpr uint8_t MODE_16TH = 0x4; // 1  0  0
static constexpr uint8_t MODE_32TH = 0x5; // 1  0  1

// Accelaration/Decelaration control
// Speed Up/Down after approx. 500msec.
static constexpr uint16_t MAX_STEPS_ARR[] = {UINT16_MAX, 128, 64, 32, 16, 8};

// Filled in distributeSteps() function
// Steps in mode32, mode16, ..., modeHalf, modeFull, modeHalf, ... , mode32
static uint16_t g_rampUpDown[11] = { 0 };

// Globals
static uint16_t g_position     = 0;
static uint16_t g_maxPosition  = 10000; // Max position of the focuser. Can be updated using SET_MAX_POS or SET_MAX_POS_VALUE commands
static uint8_t g_direction     = DIR_IN;
static uint8_t g_mode          = MODE_FULL;


// The setup() function runs once each time the micro-controller starts
void setup()
{
    digitalWrite(SLP_, LOW); // Keep DRV8825 in sleep mode at start

    // // initialize digital pin LED_BUILTIN as an output.
    // pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PUSH_IN, INPUT_PULLUP);
    pinMode(PUSH_OUT, INPUT_PULLUP);
    pinMode(FAULT_, INPUT);
    pinMode(DIR, OUTPUT);
    pinMode(STEP, OUTPUT);
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(SLP_, OUTPUT);

    Serial.begin(57600);
    Serial.flush();
 }

void setDirection(uint8_t direction)
{
    g_direction = (direction == DIR_OUT) ? DIR_OUT : DIR_IN;
    digitalWrite(DIR, g_direction);
}

void setMode(uint8_t mode)
{
    g_mode = (mode <= MODE_32TH) ? mode : MODE_32TH;
    digitalWrite(M0, (g_mode & 0x1));
    digitalWrite(M1, (g_mode & 0x2) >> 1);
    digitalWrite(M2, (g_mode & 0x4) >> 2);
}

uint16_t oneStep(uint16_t t)
{
    uint16_t numMicroSteps = 1 << g_mode;
    for (uint16_t i = 0; i < numMicroSteps; ++i)
    {
        digitalWrite(STEP, HIGH);
        delay(t);
        digitalWrite(STEP, LOW);
        delay(t);
    }
    if (g_direction == DIR_IN)
    {
        g_position = (g_position != 0) ? (g_position - 1) : 0;
        // g_position is unsigned. Still (g_position > 0) is giving FALSE if g_position > 32767 (0x7fff)
        // Hence using (g_position != 0) check
    }
    else
    {
        g_position++;
    }

    if (Serial.available() > 0)
    {
        String cmd = Serial.readStringUntil('#');
        if (cmd == "HALT")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd : HALT ");
#endif
            Serial.print(g_position); Serial.println("#");
            return -1;
        }
        else
        {
            Serial.println("\nIgnoring command while moving..");
            Serial.print(cmd); Serial.println("#");
        }
    }
    return 0;
}

void distributeSteps(uint16_t numSteps)
{
    memset(g_rampUpDown, 0, sizeof(g_rampUpDown));
    /* Distribute steps as below:
    | Up32 | Up16 | Up8 | Up4 | Up2 | FullStep | Down2 | Down4 | Down8 | Down16 | Down32 |
    Lowest speeds are filled first. Whatever remains is in FullStep mode.
    */
    for (uint16_t mode = MODE_32TH; mode >= MODE_HALF; mode--)
    {
        if (numSteps >= MAX_STEPS_ARR[mode])
        {
            g_rampUpDown[MODE_32TH - mode] = MAX_STEPS_ARR[mode];
            numSteps -= MAX_STEPS_ARR[mode];
        }
        else
        {
            g_rampUpDown[MODE_32TH - mode] = numSteps;
            numSteps = 0;
            break;
        }

        if (numSteps >= MAX_STEPS_ARR[mode])
        {
            g_rampUpDown[MODE_32TH + mode] = MAX_STEPS_ARR[mode];
            numSteps -= MAX_STEPS_ARR[mode];
        }
        else
        {
            g_rampUpDown[MODE_32TH + mode] = numSteps;
            numSteps = 0;
            break;
        }
    }
    g_rampUpDown[MODE_32TH] = numSteps;
#if DEBUG_PRINTS
    Serial.println("\nDistribute steps:");
    for (int i = 0; i <= 10; i++)
    {
        Serial.print(g_rampUpDown[i]);
        Serial.println("\n");
    }
#endif
}

void move(uint8_t direction, uint16_t numSteps)
{
    if (numSteps == 0)
    {
        return;
    }
    digitalWrite(SLP_, HIGH); // Enable DRV8825
    setDirection(direction);
    if ((direction == DIR_IN) && (numSteps > g_position))
    {
        numSteps = g_position; // Stop at 0
#if DEBUG_PRINTS
        Serial.println("\nWARNING! Will stop moving IN at 0");
#endif
    }
    else if ((direction == DIR_OUT) && ((g_position + numSteps) > g_maxPosition))
    {
        numSteps = g_maxPosition - g_position; // Stop at maxPosition
#if DEBUG_PRINTS
        Serial.println("\nWARNING! Will stop moving OUT at maxPosition");
#endif
    }
    distributeSteps(numSteps);

    // RampUp
    // | Up32 | Up16 | Up8 | Up4 | Up2 | FullStep |
    for (uint16_t i = 0; i <= MODE_32TH; i++)
    {
        uint16_t t = 1; // (i == 0) ? 5 : 1;
        setMode(MODE_32TH - i);
        for (uint16_t step = 0; step < g_rampUpDown[i]; step++)
        {
            if (oneStep(t) != 0)
            {
                return;
            }
        }
    }
    // RampDown
    // | Down2 | Down4 | Down8 | Down16 | Down32 |
    for (uint16_t i = MODE_32TH + 1; i <= 2*MODE_32TH; i++)
    {
        uint16_t t = 1; //  (i == 0) ? 5 : 1;
        setMode(i - MODE_32TH);
        for (uint16_t step = 0; step < g_rampUpDown[i]; step++)
        {
            if (oneStep(t) != 0)
            {
                return;
            }
        }
    }

    delay(50); // Some delay before disabling
    digitalWrite(SLP_, LOW); // Disable DRV8825
}

// Add the main program code into the continuous loop() function
void loop()
{
    if (digitalRead(PUSH_IN) == 0)
    {
#if DEBUG_PRINTS
        Serial.println("\nManual IN");
#endif
        digitalWrite(SLP_, HIGH); // Enable DRV8825
        setDirection(DIR_IN);
        uint16_t mode = MODE_32TH;
        setMode(mode);
        uint16_t stepsInThisMode = 0;
        do
        {
            oneStep(1);
            if (mode != MODE_FULL)
            {
                stepsInThisMode++;
                if (stepsInThisMode >= MAX_STEPS_ARR[mode])
                {
                    mode--;
                    setMode(mode);
                    stepsInThisMode = 0;
                }
            }
        } while (digitalRead(PUSH_IN) == 0);
        delay(50); // Some delay before disabling
        digitalWrite(SLP_, LOW); // Disable DRV8825
#if DEBUG_PRINTS
        Serial.println("\nDONE. Current position:");
#endif
        Serial.print(g_position); Serial.println("#");
    }
    if (digitalRead(PUSH_OUT) == 0)
    {
#if DEBUG_PRINTS
        Serial.println("\nManual OUT");
#endif
        digitalWrite(SLP_, HIGH); // Enable DRV8825
        setDirection(DIR_OUT);
        uint16_t mode = MODE_32TH;
        setMode(mode);
        uint16_t stepsInThisMode = 0;
        do
        {
            oneStep(1);
            if (mode != MODE_FULL)
            {
                stepsInThisMode++;
                if (stepsInThisMode >= MAX_STEPS_ARR[mode])
                {
                    mode--;
                    setMode(mode);
                    stepsInThisMode = 0;
                }
            }
        } while (digitalRead(PUSH_OUT) == 0);
        delay(50); // Some delay before disabling
        digitalWrite(SLP_, LOW); // Disable DRV8825
#if DEBUG_PRINTS
        Serial.println("\nDONE. Current position:");
#endif
        Serial.print(g_position); Serial.println("#");
    }

    String cmd;
    if (Serial.available() > 0)
    {
        cmd = Serial.readStringUntil('#');
        if (cmd == "OUT")
        {
            long value = Serial.parseInt();
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: OUT ");
            Serial.print(value);
#endif
            move(DIR_OUT, value);
#if DEBUG_PRINTS
            Serial.println("\nDONE OUT. Current position:");
#endif
            Serial.print(g_position); Serial.println("#");
        }
        else if (cmd == "IN")
        {
            long value = Serial.parseInt();
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: IN ");
            Serial.print(value);
#endif
            move(DIR_IN, value);
#if DEBUG_PRINTS
            Serial.println("\nDONE IN. Current position:");
#endif
            Serial.print(g_position); Serial.println("#");
        }
        else if (cmd == "SET_POS")
        {
            long value = Serial.parseInt();
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: SET_POS ");
            Serial.print(value);

            Serial.println("\nMoving focuser from position ");
            Serial.print(g_position);
            Serial.println("\n to new position ");
            Serial.print(value);
#endif
            if (value > g_position)
            {
                move(DIR_OUT, value - g_position);
            }
            else
            {
                move(DIR_IN, g_position - value);
            }
#if DEBUG_PRINTS
            Serial.println("\nDONE SETPOS. Current position:");
#endif
            Serial.print(g_position); Serial.println("#");
        }
        else if (cmd == "GET_POS")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: GET_POS");
            Serial.println("\nPosition:");
#endif
            Serial.print(g_position); Serial.println("#");
        }
        else if (cmd == "SET_ZERO_POS")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: SET_ZERO_POS");
            Serial.println("\nOld position was:");
            Serial.print(g_position);
#endif
            g_position = 0;
#if DEBUG_PRINTS
            Serial.println("\nNew position is:");
#endif
            Serial.print(g_position); Serial.println("#");
        }
        else if (cmd == "SET_MAX_POS")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: SET_MAX_POS");
            Serial.println("\nOld maxPosition was:");
            Serial.print(g_maxPosition);
#endif
            g_maxPosition = g_position;
#if DEBUG_PRINTS
            Serial.println("\nNew maxPosition:");
#endif
            Serial.print(g_maxPosition); Serial.println("#");
        }
        else if (cmd == "SET_MAX_POS_VALUE")
        {
            long value = Serial.parseInt();
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: SET_MAX_POS_VALUE");
            Serial.print(value);
            Serial.println("\nOld maxPosition was:");
            Serial.print(g_maxPosition);
#endif
            g_maxPosition = value;
#if DEBUG_PRINTS
            Serial.println("\nNew maxPosition:");
#endif
            Serial.print(g_maxPosition); Serial.println("#");
        }
        else if (cmd == "GET_MAX_POS_VALUE")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd: GET_MAX_POS_VALUE");
            Serial.println("\nmaxPosition:");
#endif
            Serial.print(g_maxPosition); Serial.println("#");
        }
        else if (cmd == "HALT")
        {
#if DEBUG_PRINTS
            Serial.println("\nSerial cmd : HALT ");
            Serial.println("\nAlready halted");
#endif
            Serial.print(g_position); Serial.println("#");
        }
#if DEBUG_PRINTS
        else if (cmd == "TEST")
        {
            Serial.println("\nSerial cmd : TEST ");
            digitalWrite(SLP_, 1);
            setMode(MODE_FULL);
            setDirection(DIR_OUT);
            oneStep(1);
            Serial.println("\nDONE#");

        }
#endif
        else
        {
            Serial.println("\nINVALID CMD#");
        }
    }
}
