/*
teensy_control_tl_cam/src/main.cpp
2022-07-26 Created by: Wataru Ito

Function:
    Generates GPIO output to trigger the cameras.
Development env:
    PlatformIO / VScode
    Open the "teensy_control_tl_cam.code-workspace"
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
#include <SimpleCLI.h>

/*############################################################################
Initialize Teensy Audio Library
    See
    Teensy Audio Library
    https://www.pjrc.com/teensy/td_libs_Audio.html
############################################################################*/
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioSynthNoiseWhite noise1;  // xy=164.00000381469727,379.0000925064087
AudioSynthWaveform waveform1; // xy=173,428.00001430511475
AudioMixer4 mixer1;           // xy=357.00000762939453,397.00001335144043
AudioOutputI2S i2s1;          // xy=512.0000152587891,397.00001335144043
AudioConnection patchCord1(noise1, 0, mixer1, 0);
AudioConnection patchCord2(waveform1, 0, mixer1, 1);
AudioConnection patchCord3(mixer1, 0, i2s1, 0);
AudioConnection patchCord4(mixer1, 0, i2s1, 1);
AudioControlSGTL5000 sgtl5000_1; // xy=345.0000762939453,495.0000400543213

/*############################################################################
Initialize NeoPixel
    See
    Adafruit NeoPixel Userguide / Arduino Library Use
    https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use
############################################################################*/
#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN 32
// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 8
// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50 // Set BRIGHTNESS to about 1/5 (max = 255)
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

/*############################################################################
Initialize SimpleCLI command line interface
    First, see the void command_setup() for each command and arguments
    Then, see each callback function for details
############################################################################*/
SimpleCLI cli;
// set LED strip ##################################
Command cmdSetLed; // Command "set_led"
// Set default color for each LED
uint8_t rgbw_value_stored[8][4] = {
    {255, 0, 0, 0},
    {0, 255, 0, 0},
    {0, 0, 255, 0},
    {0, 0, 0, 255},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}};
#define LED_BLINK_DUR_IN_MILLI 100 // LED blinking at 5 Hz, 100 milliseconds * 2 intervals
#define NEO_PIXEL_CAMERA 0         // NeoPixel LED indicator for camera
#define NEO_PIXEL_FZ_SIMU 1        // NeoPixel LED indicator for freezing simulation
unsigned long led_start = 0;       // monitor elapsed time after LED starts blinking cycle
bool led_on = LOW;                 // LED status
uint8_t neo_pixel_status[8] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
unsigned long neo_pixel_start = 0; // common timer for blinking of all NeoPixel LEDs
// set verbose mode ##################################
Command cmdSetVerbose; // Command "set_verbose"
bool verboseStatus = true;

// control camera trigger ############################
Command cmdSetCamera; // Command "set_camera"
// camera control
int fps = 30;                            // Default fps: 30
float exposure = 0.02;                   // Default exposure time: 20 ms
float cycle_duration = 1.0 / float(fps); // Default cycle duration: 1/30 sec

int gain = 1; // Default gain: 1

bool camera_on = false;            // actual camera status
bool camera_shutter_open = false;  // camera shutter status
bool camera_on_set = false;        // set_camera command status
unsigned long camera_on_start = 0; // monitor elapsed time after simulation starts

// digital pin assignment
#define PIN_CAMERA_TRIG_OUT 31 // camera trigger generating pin
#define PIN_CAMERA_TRIG_IN 33  // Control camera start/stop

// paradigm manager ##################################
Command cmdSetParadigm; // Command "set_paradigm"
//      time values in the paradigm table are in millisecond.
String paradigmName;
long none = -1;
long paradigmStart = 0;
long paradigmStartTime = 0;
long paradigmEnd = none;
long tone_start[5] = {none, none, none, none, none};
long tone_end[5] = {none, none, none, none, none};
long shocker_start[5] = {none, none, none, none, none};
long shocker_end[5] = {none, none, none, none, none};

// digital out simulated freezing bouts #############
Command cmdSetFzSimulator; // Command "set_fz_simulator"
#define DUR_CS 120.0       // The duration of CS, 120.0 seconds
#define CS_START 241       // 241th frame is the first video frame.

#define PIN_FZ_SIMU_TRIG_OUT 13 // output for simulated freezing bouts
#define PIN_FZ_SIMU_TRIG_IN 34  // input from FreezeFrame

#define SELECT_FREEZE_0 30 // select fz_start_frame_0
#define SELECT_FREEZE_1 31 // select fz_start_frame_1
#define SELECT_FREEZE_2 32 // select fz_start_frame_2

#define ARRAY_SIZE 100 // max size of fz_start, fz_end array

// Freezing simulation data
// 20190408_testing_1_m10ab subject1
int fz_start_frames_0[] = {266, 291, 296, 312, 335, 348, 372, 388, 454, 466, 493, 510, 533, 549, 579, 605, 626, 682, 702};
int fz_end_frames_0[] = {282, 295, 303, 327, 344, 366, 387, 437, 462, 486, 506, 526, 542, 572, 596, 622, 641, 692, 715};
// 20190408_testing_1_m6ab, subjects 1
int fz_start_frames_1[] = {258, 272, 336, 364, 400, 434, 465, 486, 504, 543, 560, 578, 596, 628, 714};
int fz_end_frames_1[] = {262, 323, 350, 395, 410, 457, 484, 499, 535, 552, 573, 591, 618, 655, 720};
// 20190408_testing_1_m6ab, subjects 2
int fz_start_frames_2[] = {245, 257, 276, 299, 327, 372, 433, 442, 516, 580, 589, 628, 699};
int fz_end_frames_2[] = {249, 268, 285, 318, 359, 407, 439, 507, 572, 588, 622, 676, 710};

int freeze_model = -1; // selected freezing simulation data
int size_fz_start;     // size of fz_start
int size_fz_end;       // size of fz_end

float fz_start[ARRAY_SIZE];
float fz_end[ARRAY_SIZE];

unsigned long fz_simulation_start_time = 0; // monitor elapsed time after simulation starts
bool fz_simulation_on = false;              // freezing simulation is on
bool fz_epoch_on = false;                   // freezing epoch is on

// set tone #####################################
Command cmdSetTone;       // Command "set_tone"
bool tone_on_set = false; // set_tone command status
int tone_freq = 0;        // Default tone frequency
float tone_amp = 0.0;     // Default tone amplitude

// set noise #####################################
Command cmdSetNoise;       // Command "set_noise"
bool noise_on_set = false; // set_noise command status
float noise_amp = 0.0;     // Default noise amplitude

// start paradigm ###################################
// Command cmdStart;

/*############################################################################
constants and variables
############################################################################*/
// parameters for controlling
#define SERIAL_TIMEOUT 100000 // Serial connection timeout in milliseconds

/*############################################################################
Create IntervalTimer object
    https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
############################################################################*/
#define TIME_SAMPLING_FREQ 100000 // Timer is set at the maximum (100k Hz).
#define SEC_IN_MICRO 1000000.0    // constant: 1 second in microseconds
IntervalTimer int_timer;

/*############################################################################
SimpleCLI command interpreter callback functions
############################################################################*/
// Command "set_led" details
void set_ledCallback(cmd *cmdPtr)
{
    String rgbw[] = {"r", "g", "b", "w"};
    uint8_t rgbw_value[4];

    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("led");
    int _led_n = arg.getValue().toInt();

    for (int i = 0; i < 4; i++)
    {
        arg = cmd.getArgument(rgbw[i]);
        if (arg.isSet())
        {
            rgbw_value[i] = arg.getValue().toInt();
        }
        else
        {
            rgbw_value[i] = 0;
        }
    }

    // Store LED strip data
    for (int i = 0; i < 4; i++)
    {
        rgbw_value_stored[_led_n][i] = rgbw_value[i];
    }
    //  Set pixel's color (in RAM)
    strip.setPixelColor(_led_n, strip.Color(rgbw_value_stored[_led_n][0], rgbw_value_stored[_led_n][1],
                                            rgbw_value_stored[_led_n][2], rgbw_value_stored[_led_n][3]));
    strip.show();
}

// Command "set_verbose" details
void set_verboseCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("on");
    bool _on = arg.isSet();
    arg = cmd.getArgument("off");
    bool _off = arg.isSet();

    if (_on)
    {
        verboseStatus = true;
        Serial.println("Verbose mode ON");
    }
    if (_off)
    {
        verboseStatus = false;
        Serial.println("Verbose mode OFF");
    }
}

// Command "set_paradigm" details
void set_paradigmCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);

    Argument arg = cmd.getArgument("tone");
    bool _paradigmTone = arg.isSet();
    //  Serial.print("_paradigmTone: ");
    //  Serial.println(_paradigmTone);

    arg = cmd.getArgument("shocker");
    bool _paradigmShocker = arg.isSet();

    arg = cmd.getArgument("clear");
    bool _paradigmClear = arg.isSet();

    arg = cmd.getArgument("list");
    bool _paradigmList = arg.isSet();
    //  Serial.print("_paradigmList: ");
    //  Serial.println(_paradigmList);

    arg = cmd.getArgument("epoch");
    int _paradigmEpoch = arg.getValue().toInt();

    arg = cmd.getArgument("start");
    float _paradigmStart = arg.getValue().toFloat();

    arg = cmd.getArgument("end");
    float _paradigmEnd = arg.getValue().toFloat();

    arg = cmd.getArgument("paradigm_duration");
    float _paradigmDuration = arg.getValue().toFloat();

    arg = cmd.getArgument("paradigm_name");
    String _paradigmName = arg.getValue();

    if (_paradigmName != "")
    {
        paradigmName = _paradigmName;
    }

    if (_paradigmDuration != -1.0)
    {
        paradigmEnd = long(_paradigmDuration * 1000.0);
        //    Serial.print("paradigmDuration: ");
        //    Serial.println(_paradigmDuration);
    }
    if (_paradigmEpoch != -1)
    {
        if (_paradigmTone)
        {
            tone_start[_paradigmEpoch] = long(_paradigmStart * 1000.0);
            tone_end[_paradigmEpoch] = long(_paradigmEnd * 1000.0);
            //      Serial.print("tone: ");
            //      Serial.print(_paradigmEpoch);
            //      Serial.print(" ");
            //      Serial.print(_paradigmStart);
            //      Serial.print(" ");
            //      Serial.println(_paradigmEnd);
        }
        if (_paradigmShocker)
        {
            shocker_start[_paradigmEpoch] = long(_paradigmStart * 1000.0);
            shocker_end[_paradigmEpoch] = long(_paradigmEnd * 1000.0);
            //      Serial.print("shocker: ");
            //      Serial.print(_paradigmEpoch);
            //      Serial.print(" ");
            //      Serial.print(_paradigmStart);
            //      Serial.print(" ");
            //      Serial.println(_paradigmEnd);
        }
    }
    if (_paradigmClear)
    {
        paradigmEnd = -1;
        for (int i = 0; i < 5; i++)
        {
            tone_start[i] = -1;
            tone_end[i] = -1;
            shocker_start[i] = -1;
            shocker_end[i] = -1;
        }
    }
    if (_paradigmList)
    {
        Serial.print("paradigm name: ");
        Serial.println(paradigmName);
        Serial.print("paradigm duration: ");
        Serial.print(paradigmEnd / 1000.0);
        Serial.println(" sec");
        for (int i = 0; i < 5; i++)
        {
            Serial.print("tone epoch ");
            Serial.print(i);
            Serial.print(" tone_start: ");
            Serial.print(tone_start[i] / 1000.0);
            Serial.print(" sec, tone_end: ");
            Serial.print(tone_end[i] / 1000.0);
            Serial.println(" sec");
        }
        for (int i = 0; i < 5; i++)
        {
            Serial.print("shocker epoch ");
            Serial.print(i);
            Serial.print(" shocker_start: ");
            Serial.print(shocker_start[i] / 1000.0);
            Serial.print(" sec, shocker_end: ");
            Serial.print(tone_end[i] / 1000.0);
            Serial.println(" sec");
        }
    }
}

// Command "set_camera" details
//      Check fps and exposure time are valid
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

// Command "set_fz_simulator" details
// Convert from frame to seconds
void conv_frame_to_sec(int start_end_frames[], float start_end[], int size_array)
{
    int i;
    for (i = 0; i < size_array; i++)
    {
        start_end[i] = float(start_end_frames[i] - CS_START) / 4.0;
    }
}
void set_fz_simulatorCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("show");
    // bool _set_fz_simulatorStatus = arg.isSet();
    // if (_set_fz_simulatorStatus)
    // bool _set_fz_simulatorStatus = arg.isSet();
    if (arg.isSet())
    {
        Serial.println("### freeze simulator ###");
        Serial.print("Current model is: ");
        Serial.println(freeze_model);
        Serial.print("size_fz_start is: ");
        Serial.println(size_fz_start);
        Serial.print("size_fz_end is: ");
        Serial.println(size_fz_end);
        Serial.println("Stored freezing patterns:");
        Serial.print("fz_start (frame): ");
        for (int i = 0; i < size_fz_start; i++)
        {
            Serial.print(fz_start[i]);
            Serial.print(" ");
        }
        Serial.println("");
        Serial.print("fz_end (frame): ");
        for (int i = 0; i < size_fz_start; i++)
        {
            Serial.print(fz_end[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }

    arg = cmd.getArgument("model");
    if (arg.isSet())
    {
        int _model = arg.getValue().toInt();

        if (_model >= 0 && _model <= 3)
        {
            freeze_model = _model;

            switch (freeze_model)
            {
            case 0:
                // sizeof returns the size of the array in bytes.
                // Divide by the size of the first element to get the number of elements.
                size_fz_start = sizeof(fz_start_frames_0) / sizeof(fz_start_frames_0[0]);
                size_fz_end = sizeof(fz_end_frames_0) / sizeof(fz_end_frames_0[0]);

                conv_frame_to_sec(fz_start_frames_0, fz_start, size_fz_start);
                conv_frame_to_sec(fz_end_frames_0, fz_end, size_fz_end);
                break;
            case 1:
                size_fz_start = sizeof(fz_start_frames_1) / sizeof(fz_start_frames_1[0]);
                size_fz_end = sizeof(fz_end_frames_1) / sizeof(fz_end_frames_1[0]);

                conv_frame_to_sec(fz_start_frames_1, fz_start, size_fz_start);
                conv_frame_to_sec(fz_end_frames_1, fz_end, size_fz_end);
                break;
            case 2:
                size_fz_start = sizeof(fz_start_frames_2) / sizeof(fz_start_frames_2[0]);
                size_fz_end = sizeof(fz_end_frames_2) / sizeof(fz_end_frames_2[0]);

                conv_frame_to_sec(fz_start_frames_2, fz_start, size_fz_start);
                conv_frame_to_sec(fz_end_frames_2, fz_end, size_fz_end);
                break;
            }
        }
        else
        {
            Serial.println("Invalid model number");
        }
    }
}

// Command "set_tone" details
void tone_show_status()
{
    Serial.print("Tone status: ");
    Serial.print(tone_on_set);
    Serial.print(", freq: ");
    Serial.print(tone_freq);
    Serial.print(" Hz, amp: ");
    Serial.println(tone_amp);
}
void set_toneCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("start");
    if (arg.isSet())
    {
        tone_on_set = true;
    }

    arg = cmd.getArgument("stop");
    if (arg.isSet())
    {
        tone_on_set = false;
    }

    arg = cmd.getArgument("freq");
    int _freq = arg.getValue().toInt();
    if (_freq != -1)
    {
        tone_freq = _freq;
    }

    arg = cmd.getArgument("amp");
    float _amp = arg.getValue().toFloat();
    if (_amp != -1)
    {
        tone_amp = _amp;
    }

    if (tone_on_set)
    {
        waveform1.frequency(tone_freq);
        waveform1.amplitude(tone_amp);
    }
    else
    {
        waveform1.amplitude(0);
    }
    tone_show_status();
}

// Command "set_noise" details
void noise_show_status()
{
    Serial.print("Noise status: ");
    Serial.print(noise_on_set);
    Serial.print(", amp: ");
    Serial.println(noise_amp);
}
void set_noiseCallback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument("start");
    if (arg.isSet())
    {
        noise_on_set = true;
    }

    arg = cmd.getArgument("stop");
    if (arg.isSet())
    {
        noise_on_set = false;
    }

    arg = cmd.getArgument("amp");
    float _amp = arg.getValue().toFloat();
    if (_amp != -1)
    {
        noise_amp = _amp;
    }

    if (noise_on_set)
    {
        noise1.amplitude(noise_amp);
    }
    else
    {
        noise1.amplitude(0);
    }
    noise_show_status();
}

// Error processing
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

// Defining commands
void command_setup()
{
    // Register commands for Command line interface

    // Command "set_led"
    cmdSetLed = cli.addCommand("set_led", set_ledCallback);
    cmdSetLed.addArgument("led", "-1");
    cmdSetLed.addArgument("r", "0");
    cmdSetLed.addArgument("g", "0");
    cmdSetLed.addArgument("b", "0");
    cmdSetLed.addArgument("w", "0");

    // Command "set_verbose"
    cmdSetVerbose = cli.addCommand("set_verbose", set_verboseCallback);
    cmdSetVerbose.addFlagArgument("on");
    cmdSetVerbose.addFlagArgument("off");

    // Command "set_paradigm"
    cmdSetParadigm = cli.addCommand("set_paradigm", set_paradigmCallback);
    cmdSetParadigm.addFlagArgument("tone");
    cmdSetParadigm.addFlagArgument("shocker");
    cmdSetParadigm.addFlagArgument("clear");
    cmdSetParadigm.addFlagArgument("list");
    cmdSetParadigm.addArgument("epoch", "-1");
    cmdSetParadigm.addArgument("start", "-1");
    cmdSetParadigm.addArgument("end", "-1");
    cmdSetParadigm.addArgument("paradigm_duration", "-1");
    cmdSetParadigm.addArgument("paradigm_name", "");

    // Command "set_camera"
    cmdSetCamera = cli.addCommand("set_camera", set_cameraCallback);
    cmdSetCamera.addFlagArgument("start");
    cmdSetCamera.addFlagArgument("stop");
    cmdSetCamera.addFlagArgument("status");
    cmdSetCamera.addArgument("fps", "-1");
    cmdSetCamera.addArgument("exposure", "-1.0");

    // Command "set_fz_simulator"
    cmdSetFzSimulator = cli.addCommand("set_fz_simulator", set_fz_simulatorCallback);
    cmdSetFzSimulator.addFlagArgument("show");
    cmdSetFzSimulator.addArgument("model", "-1");

    // Command "set_tone"
    cmdSetTone = cli.addCommand("set_tone", set_toneCallback);
    cmdSetTone.addFlagArgument("start");
    cmdSetTone.addFlagArgument("stop");
    cmdSetTone.addArgument("freq", "-1");
    cmdSetTone.addArgument("amp", "-1");

    // Command "set_noise"
    cmdSetNoise = cli.addCommand("set_noise", set_noiseCallback);
    cmdSetNoise.addFlagArgument("start");
    cmdSetNoise.addFlagArgument("stop");
    cmdSetNoise.addArgument("amp", "-1");

    // In case of error
    cli.setOnError(errorCallback);
}

/*############################################################################
// Functions called from interval_timer started in void setup()
############################################################################*/
// ##### Control NeoPixel LED
void led_control(int _led_n, uint8_t led_on)
{
    if (led_on == HIGH)
    { // LED on
        if (neo_pixel_status[_led_n] == LOW)
        {
            strip.setPixelColor(_led_n, strip.Color(rgbw_value_stored[_led_n][0], rgbw_value_stored[_led_n][1],
                                                    rgbw_value_stored[_led_n][2], rgbw_value_stored[_led_n][3]));
            neo_pixel_status[_led_n] = HIGH;
            strip.show();
        }
    }
    else
    { // LED off
        if (neo_pixel_status[_led_n] == HIGH)
        {
            strip.setPixelColor(_led_n, strip.Color(0, 0, 0, 0));
            neo_pixel_status[_led_n] = LOW;
            strip.show();
        }
    }
}
// Control the LED blinking
void led_blink(int _led_n)
{
    unsigned long time_now = millis();
    if (time_now - neo_pixel_start > LED_BLINK_DUR_IN_MILLI * 2)
    {
        led_on = HIGH;
        neo_pixel_start = time_now;
    }
    else if (time_now - neo_pixel_start > LED_BLINK_DUR_IN_MILLI)
    {
        led_on = LOW;
    }

    led_control(_led_n, led_on);
}

// ##### Control the camera
void generate_camera_trigger()
{
    // Monitor PIN_CAMERA_TRIG_IN
    if ((digitalRead(PIN_CAMERA_TRIG_IN) == HIGH) || (camera_on_set))
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

    // Scan fz array and set PIN_FZ_SIMU_TRIG_OUT
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
                if (!camera_shutter_open)
                {
                    digitalWrite(PIN_CAMERA_TRIG_OUT, HIGH);
                    led_control(NEO_PIXEL_CAMERA, HIGH);
                    neo_pixel_status[NEO_PIXEL_CAMERA] = HIGH;
                    camera_shutter_open = true;
                }
            }
            else
            {
                if (camera_shutter_open)
                {
                    digitalWrite(PIN_CAMERA_TRIG_OUT, LOW);
                    led_control(NEO_PIXEL_CAMERA, LOW);
                    neo_pixel_status[NEO_PIXEL_CAMERA] = LOW;
                    camera_shutter_open = false;
                }
            }
        }
        else
        {
            camera_shutter_open = true;
            camera_on_start = micros();
            digitalWrite(PIN_CAMERA_TRIG_OUT, HIGH);
            led_control(NEO_PIXEL_CAMERA, HIGH);
            neo_pixel_status[NEO_PIXEL_CAMERA] = HIGH;
        }
    }
    else
    { // when camera is false, turn off the camera and blink LED.
        digitalWrite(PIN_CAMERA_TRIG_OUT, LOW);
        led_blink(NEO_PIXEL_CAMERA);
    }
}

// ##### fz simulator
// Return the index of the nearest earlier events
int current_time_is_in_epoch(float x, float start[], float end[], int size_array)
{
    for (int i = 0; i < size_array; i++)
    {
        if ((x >= start[i]) && (x <= end[i]))
        {
            return i;
        }
    }

    return -1;
}

void simulated_fz()
{
    // Monitor PIN_FZ_SIMU_TRIG_IN
    if (digitalRead(PIN_FZ_SIMU_TRIG_IN) == HIGH)
    {
        if (!fz_simulation_on) // freeze simulation starts
        {
            fz_simulation_start_time = micros();
            digitalWrite(PIN_FZ_SIMU_TRIG_OUT, LOW);
            led_control(NEO_PIXEL_FZ_SIMU, LOW);
            neo_pixel_status[NEO_PIXEL_FZ_SIMU] = LOW;
            fz_epoch_on = false;
        }
        fz_simulation_on = true;
    }
    else
    {
        fz_simulation_on = false;
    }

    // Scan fz array and set PIN_FZ_SIMU_TRIG_OUT
    if (fz_simulation_on)
    {
        unsigned long time_current = micros();
        float elapsed_time = float(time_current - fz_simulation_start_time) / SEC_IN_MICRO;

        // Serial.print("time_current_f: ");
        // Serial.println(time_current_f);

        if (elapsed_time < DUR_CS)
        {
            int fz_index = current_time_is_in_epoch(elapsed_time, fz_start, fz_end, size_fz_start);

            if (fz_index >= 0) // current_time is in epoch
            {
                if (!fz_epoch_on)
                {
                    digitalWrite(PIN_FZ_SIMU_TRIG_OUT, HIGH);
                    led_control(NEO_PIXEL_FZ_SIMU, HIGH);
                    neo_pixel_status[NEO_PIXEL_FZ_SIMU] = HIGH;
                    fz_epoch_on = true;
                }
            }
            else // current_time is out of epoch
            {
                if (fz_epoch_on)
                {
                    digitalWrite(PIN_FZ_SIMU_TRIG_OUT, LOW);
                    led_control(NEO_PIXEL_FZ_SIMU, LOW);
                    neo_pixel_status[NEO_PIXEL_FZ_SIMU] = LOW;
                    fz_epoch_on = false;
                }
            }
        }
        else
        {
            if (fz_epoch_on)
            {
                digitalWrite(PIN_FZ_SIMU_TRIG_OUT, LOW);
                fz_epoch_on = false;
            }
            led_blink(NEO_PIXEL_FZ_SIMU);
        }
    }
    else
    {
        if (fz_epoch_on)
        {
            digitalWrite(PIN_FZ_SIMU_TRIG_OUT, LOW);
            fz_epoch_on = false;
        }
        led_blink(NEO_PIXEL_FZ_SIMU);
    }
}

// IntervalTimer callback function
void interval_timer_callback()
{
    generate_camera_trigger();
    simulated_fz();
    // pardigm_control();
}

/*############################################################################
// Functions called from void setup()
############################################################################*/
void init_pins()
{
    // initialize digital pins.
    pinMode(PIN_CAMERA_TRIG_IN, INPUT);
    pinMode(PIN_CAMERA_TRIG_OUT, OUTPUT);
    pinMode(PIN_FZ_SIMU_TRIG_IN, INPUT);
    pinMode(PIN_FZ_SIMU_TRIG_OUT, OUTPUT);

    digitalWrite(PIN_CAMERA_TRIG_OUT, LOW);
    digitalWrite(PIN_FZ_SIMU_TRIG_OUT, LOW);
}

void init_neopixel()
{
    strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();  // Turn OFF all pixels ASAP
    strip.setBrightness(BRIGHTNESS);
}

void init_fz_simulator_model()
{
    freeze_model = 0;
    // sizeof returns the size of the array in bytes.
    // Divide by the size of the first element to get the number of elements.
    size_fz_start = sizeof(fz_start_frames_0) / sizeof(fz_start_frames_0[0]);
    size_fz_end = sizeof(fz_end_frames_0) / sizeof(fz_end_frames_0[0]);

    conv_frame_to_sec(fz_start_frames_0, fz_start, size_fz_start);
    conv_frame_to_sec(fz_end_frames_0, fz_end, size_fz_end);
}

void init_teensy_audio()
{
    AudioMemory(10);
    waveform1.begin(WAVEFORM_SINE);
    delay(1000);
}

//############################################################################
void setup()
{
    init_pins();               // Initialize pins
    init_neopixel();           // Initialize neopixel
    init_fz_simulator_model(); // Initialize fz simulator model
    init_teensy_audio();       // Initialize teensy audio

    // Initialize the USB serial connection with the PC at 115200 baud
    Serial.setTimeout(SERIAL_TIMEOUT);
    Serial.begin(115200);
    // Wait for the serial connection to be established
    delay(2000);

    // Initialize the command line interface
    command_setup();

    // Generate callback for the timer
    int timer_micros = 1000000 / TIME_SAMPLING_FREQ;
    int_timer.begin(interval_timer_callback, timer_micros);

    // Greeting message
    Serial.println("Welcome to PCBox: Paradigm Controlling Box");
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

    // waveform1.frequency(8000);
    // waveform1.amplitude(0.2);
    // delay(250);
    // waveform1.amplitude(0);
    // delay(1750);
}
