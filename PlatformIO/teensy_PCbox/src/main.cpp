/*
teensy_control_tl_cam/src/main.cpp
2022-07-26 Created by: Wataru Ito

Function:
    Generates GPIO output to trigger the cameras.
Development env:
    PlatformIO/VS Code
    read the "teensy_control_tl_cam.code-workspace"
Dependency:
    SimpleCLI   https://github.com/SpacehuhnTech/SimpleCLI
*/

/*
Relevant commands
    commu_arduino("set_camera -cameras_number " + str(cameraNum))
    commu_arduino("set_camera -start")
    commu_arduino("set_camera -fps " + str(fps) + " -exposure " + str(exposure))
    commu_arduino("set_camera -status")
    commu_arduino("set_camera -stop")
    commu_arduino("start paradigm -button")
*/
#include <Arduino.h>

// Initialize SimpleCLI command line interface
#include <SimpleCLI.h>
SimpleCLI cli;
Command cmdSetCamera;
// Command cmdSetVerbose;
// Command cmdSetParadigm;
// Command cmdSetTone;
// Command cmdStart;

// parameters for controlling
#define LED_BLINK_DUR_IN_MILLI 100 // LED blinking at 5 Hz, 100 milliseconds * 2 intervals
#define SEC_IN_MICRO 1000000.0     // constant: 1 second in microseconds
#define SERIAL_TIMEOUT 100000      // Serial timeout in milliseconds
#define TIME_SAMPLING_FREQ 100000  // GPIO input/output sampling at 100k Hz

// digital pin assignment
#define PIN_CAM 12          // camera trigger generating pin
#define PIN_TRIG_IN 24      // Control camera start/stop
#define PIN_LED LED_BUILTIN // LED output pin

// global variables
bool verboseStatus = true;

// camera control
int fps = 30;                            // Default fps: 30
float exposure = 0.02;                   // Default exposure time: 20 ms
float cycle_duration = 1.0 / float(fps); // Default cycle duration: 1/30 sec

int gain = 1; // Default gain: 1

bool camera_on = false;            // actual camera status
bool camera_on_set = false;        // state of camera on/off setting
unsigned long camera_on_start = 0; // monitor elapsed time after simulation starts

// led control
unsigned long led_start = 0; // monitor elapsed time after LED starts blinking cycle
bool led_on = LOW;           // LED status

// Create IntervalTimer object
//      https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
IntervalTimer int_timer;

void print_status()
{
    Serial.print("Camera status: ");
    Serial.print(camera_on);
    Serial.print(", fps: ");
    Serial.print(fps);
    Serial.print(", exposure: ");
    Serial.print(exposure * 1000.0);
    Serial.print(" ms, cycle duration: ");
    Serial.print(cycle_duration * 1000.0);
    Serial.println(" ms");
}

// Check fps and exposure time are valid
bool check_consistency(int _fps, float _exposure)
{
    float _cycle_duration = 1.0 / float(_fps);
    if (_cycle_duration < _exposure)
    {
        Serial.println("Error: exposure time is longer than cycle duration");
        Serial.println("       fps or exposure is automatically adjusted.");
        return false;
    }
    else
    {
        return true;
    }
}

// Command "set_camera" details
void set_cameraCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("start");
    bool _set_cameraStart = arg.isSet();
    if (_set_cameraStart)
    {
        camera_on_set = true;
        // Serial.println(camera_on_set);
        print_status();
    }

    arg = cmd.getArgument("stop");
    bool _set_cameraStop = arg.isSet();
    if (_set_cameraStop)
    {
        camera_on_set = false;
        // Serial.println(camera_on_set);
    }

    arg = cmd.getArgument("status");
    bool _set_cameraStatus = arg.isSet();
    if (_set_cameraStatus)
    {
        print_status();
    }

    arg = cmd.getArgument("fps");
    int _fps = arg.getValue().toInt();
    if (_fps != -1)
    {
        if (check_consistency(_fps, exposure))
        {
            fps = _fps;
            cycle_duration = 1.0 / float(fps);
            print_status();
        }
        else
        {
            fps = _fps;
            cycle_duration = 1.0 / float(fps);
            exposure = cycle_duration * 0.5;
            print_status();
        }
    }

    arg = cmd.getArgument("exposure");
    float _exposure = arg.getValue().toFloat();
    if (_exposure != -1.0)
    {
        if (check_consistency(fps, _exposure / 1000.0))
        {
            exposure = _exposure / 1000.0;
            cycle_duration = 1.0 / float(fps);
            print_status();
        }
        else
        {
            exposure = _exposure / 1000.0;
            fps = int(1.0 / exposure);
            cycle_duration = 1.0 / float(fps);
            print_status();
        }
    }
}

void errorCallback(cmd_error *errorPtr)
{
    CommandError e(errorPtr);

    Serial.println("ERROR: " + e.toString());

    if (e.hasCommand())
    {
        Serial.println("Did you mean? " + e.getCommand().toString());
    }
    else
    {
        Serial.println(cli.toString());
    }
}

void command_setup()
{
    // Register commands for Command line interface

    // Command "set_camera"
    cmdSetCamera = cli.addCommand("set_camera", set_cameraCallback);
    cmdSetCamera.addFlagArgument("start");
    cmdSetCamera.addFlagArgument("stop");
    cmdSetCamera.addFlagArgument("status");
    cmdSetCamera.addArgument("fps", "-1");
    cmdSetCamera.addArgument("exposure", "-1.0");

    // In case of error
    cli.setOnError(errorCallback);
}

// Control the LED blinking
void led_blink()
{
    unsigned long time_now = millis();
    if (time_now - led_start > LED_BLINK_DUR_IN_MILLI * 2)
    {
        led_on = HIGH;
        led_start = time_now;
    }
    else if (time_now - led_start > LED_BLINK_DUR_IN_MILLI)
    {
        led_on = LOW;
    }

    digitalWrite(PIN_LED, led_on);
}

// IntervalTimer callback function
void interval_timer_callback()
{
    // Monitor PIN_TRIG_IN
    if ((digitalRead(PIN_TRIG_IN) == HIGH) || (camera_on_set))
    {
        if (!camera_on)
        {
            camera_on_start = micros();
        }
        camera_on = true;
    }
    else
    {
        camera_on = false;
    }

    // Scan fz array and set TRIG_OUT
    if (camera_on)
    {
        unsigned long time_current = micros();
        float elapsed_time = float(time_current - camera_on_start) / SEC_IN_MICRO;

        // Serial.print("time_current_f: ");
        // Serial.println(time_current_f);

        if (elapsed_time <= cycle_duration)
        {
            if (elapsed_time < exposure)
            {
                digitalWrite(PIN_CAM, HIGH);
                digitalWrite(PIN_LED, HIGH);
            }
            else
            {
                digitalWrite(PIN_CAM, LOW);
                digitalWrite(PIN_LED, LOW);
            }
        }
        else
        {
            camera_on_start = micros();
            digitalWrite(PIN_CAM, HIGH);
            digitalWrite(PIN_LED, HIGH);
        }
    }
    else
    {
        digitalWrite(PIN_CAM, LOW);
        led_blink();
    }
}

void setup()
{
    // initialize digital pins.
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_TRIG_IN, INPUT);
    pinMode(PIN_CAM, OUTPUT);
    digitalWrite(PIN_CAM, LOW);

    // Initialize the USB serial connection with the PC at 115200 baud
    Serial.setTimeout(SERIAL_TIMEOUT);
    Serial.begin(115200);
    // Wait for the serial connection to be established
    delay(1000);

    // Initialize the command line interface
    command_setup();

    // Generate callback for the timer
    int timer_micros = 1000000 / TIME_SAMPLING_FREQ;
    int_timer.begin(interval_timer_callback, timer_micros);
}

void loop()
{
    // Command line interface
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        if (verboseStatus)
        {
            Serial.print("> ");
            Serial.println(input);
        }
        cli.parse(input);

        // Serial.print("> ");
    }
}
