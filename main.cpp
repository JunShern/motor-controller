#include "mbed.h"
#include <string>
#include "rtos.h"
#include "PID.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//constants
#define ENCODER_INCREMENT 0.76923076923076923076923076923077
#define MAX_LINE 256
#define PID_PERIOD 0.05
//#define TARGET_SPEED 10 // (rps)
#define KP 1.0
#define KI 10000000.0
#define KD 0.000001
//#define TARGET_REVS 30
#define revKP 2.5
#define revKI 2300000.0
#define revKD 0.12
#define BIAS  0.0

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Incremental encoder inputs

DigitalIn chA (CHA);
DigitalIn chB (CHB);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

void writeAllPwms(float dc) {
    L1L.write(dc);
    L1H.write(dc);
    L2L.write(dc);
    L2H.write(dc);
    L3L.write(dc);
    L3H.write(dc);
}

// Set a given drive state
void motorOut(int8_t driveState, float dc){
    // Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    // Turn off first
    if (~driveOut & 0x01) L1L.write(0.0f); //L1L = 0;
    if (~driveOut & 0x02) L1H.write(1.0f); // = 1;
    if (~driveOut & 0x04) L2L.write(0.0f); // = 0;
    if (~driveOut & 0x08) L2H.write(1.0f); // = 1;
    if (~driveOut & 0x10) L3L.write(0.0f); // = 0;
    if (~driveOut & 0x20) L3H.write(1.0f); // = 1;   
    // Then turn on
    if (driveOut & 0x01) L1L.write(dc); // = 1;
    if (driveOut & 0x02) L1H.write(0.0f); // = 0;
    if (driveOut & 0x04) L2L.write(dc); // = 1;
    if (driveOut & 0x08) L2H.write(0.0f); // = 0;
    if (driveOut & 0x10) L3L.write(dc); // = 1;
    if (driveOut & 0x20) L3H.write(0.0f); // = 0;
}
 
   
// Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

// Basic synchronisation routine    
int8_t motorHome() {
    // Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1.0f);
    wait(3.0);
    // Get the rotor state
    return readRotorState();
}

//---------------------------------------ISR global variables
volatile int count = 0;
volatile float photo_count = 0;
bool first_measurement = true;
typedef int photo_state[3]; //declare the state of the photointerrupters as an array of boolean variables. [0] --> I1, [1] --> I2, [2] --> I3;
const photo_state photo_table[6] = {{1,0,1}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}};
photo_state old_state;
photo_state new_state;
float period = 0.0005f;
float dutyCycle = 0.5f;
volatile int regex_done = 0;
//-----------------------------------------------------------

//---------------------------------------Declare ISR funtions

int search_photo_table (photo_state state) {
    for (int i = 0; i < 6; i++){
        if (photo_table[i][0] == state[0] && photo_table[i][1] == state[1] && photo_table[i][2] == state[2]) {
            return i;
        }
    }        
    return 0; //default return value if non-existent state is detected
}
void channelA_rise () {
    if (chB == 0) count++;
    else count--;
}
void channelA_fall () {
    if (chB == 1) count++;
    else count--;
}    
void channelB_rise () {
    if (chA == 1) count ++;
    else count--;
}   
void channelB_fall () {
    if (chA ==0) count++;
    else count --;
}
    
void angle_60 () {    
    int old_index, new_index;   
    if (first_measurement) {
        photo_count = count * ENCODER_INCREMENT;
        count = 0;
        first_measurement = false;
        old_state[0] = I3.read(); 
        old_state[1] = I2.read();
        old_state[2] = I1.read();
    } else {
        new_state[0] = I3.read(); 
        new_state[1] = I2.read(); 
        new_state[2] = I1.read();
        
        new_index = search_photo_table (new_state);
        old_index = search_photo_table (old_state);
        
        if ((new_index - old_index) == + 5) {
            photo_count = -60 + photo_count;
        } else if ((new_index - old_index) == - 5) {
            photo_count = 60 + photo_count;
        } else {
            photo_count = (new_index - old_index)*60.0 + photo_count;
        }
        
        old_state[0] = new_state[0];
        old_state[1] = new_state[1];
        old_state[2] = new_state[2];
        
        count = 0;
    }
}
  
    
// --------------------------------------Declare Interrupts from photosensors
InterruptIn channelA (CHA);
InterruptIn channelB (CHB);
InterruptIn photoI1 (I1pin);
InterruptIn photoI2 (I2pin);
InterruptIn photoI3 (I3pin);
//--------------------------------------------------------------------------
    
//---------------------------------------------------Declare the Queues
Queue<uint32_t, 1> regex_queue;
Queue<uint32_t, 1> velocity_queue;
//---------------------------------------------------------------------

//---------------------------------------------------I/O
Mutex stdio_mutex;
Serial pc(SERIAL_TX, SERIAL_RX, 9600);
//--------------------------------------------------------

//-------------------------------declare the thread functions and threads
Thread regex_thread(osPriorityNormal, 1024, NULL);
Thread angle_calculation(osPriorityNormal, 1024, NULL);
Thread control_thread (osPriorityNormal, 1024, NULL);
//Thread speed_thread (osPriorityNormal, 1024, NULL);
//Thread revolution_thread (osPriorityNormal, 1024, NULL);
//--------------------------------------------------------

//----------------------------Angle calculations
volatile float live_angle = 0;
volatile float prev_angle = 0;
volatile float live_time = 0;
volatile float prev_time = 0;
Timer t;
volatile float live_velocity = 0;
//--------------------------------------------------------

//-------------------------------------------------------Declare the thread fucntions

volatile float targetRev = 0;
volatile float targetVel = 0;
volatile char controlMode = 'R';

void speed_control () {
    PID controller(KP, KI, KD, PID_PERIOD);
       
    //Analog input from 0.0 to 3.3V
    controller.setInputLimits(0.0, 100.0);
    //Pwm output from 0.0 to 1.0
    controller.setOutputLimits(0.0, 1.0);
    //If there's a bias.
    controller.setBias(0.0);
    controller.setMode(0);
    //We want the process variable to be 1.7V
    controller.setSetPoint(targetVel);
    
    while(1){
        //Update the process variable.
        controller.setProcessValue(live_velocity);
        //Set the new output.
        dutyCycle = controller.compute();
        //Wait for another loop calculation.
        wait(PID_PERIOD);
    }
}

void revolution_control () {
    PID revcontroller(revKP, revKI, revKD, PID_PERIOD);
    
    //Analog input from 0.0 to 3.3V
    revcontroller.setInputLimits(0.0, 10000.0);
    //Pwm output from 0.0 to 1.0
    revcontroller.setOutputLimits(-1.0, 1.0);
    //If there's a bias.
    revcontroller.setBias(BIAS);
    revcontroller.setMode(0);
    //We want the process variable to be 1.7V
    revcontroller.setSetPoint(targetRev);
    
    while(1){
        //Update the process variable.
        revcontroller.setProcessValue(live_angle/360.0f);
        //Set the new output.
        float controlOutput = revcontroller.compute();
        dutyCycle = abs(controlOutput);
        if (controlOutput>0) {
            lead = 2;
        } else {
            lead = -2;
        }
        //Wait for another loop calculation.
        wait(PID_PERIOD);
    }
}
    
int notesList[20];
int durationsList[20];

float noteToFreq(int ascii) {
    int note = ascii - 65; // Char 'a' in ascii -> 97
    float freq[7] = {3520.0f, 3951.1f, 2093.0f, 2349.3f, 2637.0f, 2793.0f, 3136.0f};
    return freq[note]; 
}

void playTune() {
    writeAllPwms(0.1f);
    dutyCycle = 0.1f;
    lead = 1;
    for (int i=0; i<20; i++) { 
        writeAllPwms(1.0f);
        wait(0.5);
        writeAllPwms(0.1f);
        float f = noteToFreq(notesList[i]);
        period = 1.0f/f;
        wait(durationsList[i]);
    }
}

void controllerFunction() {
    if (targetVel==0 && targetRev==0) {
        playTune();
    } else if (targetVel == 0) {
        revolution_control();
    } else if (targetRev == 0) {
        speed_control();
    } 
    return;
}

void updateAngle() {
    t.start();
    while (1) {
        prev_angle = live_angle;
        live_angle = photo_count + count * ENCODER_INCREMENT; //Degrees per second
        prev_time = live_time;
        live_time = t.read();
        live_velocity = ((live_angle - prev_angle) / (live_time-prev_time))/360.0f; //RPS
        wait(0.1);
    }
    //t.stop();
}

void regex () {
    bool invalid_command = true;
    stdio_mutex.lock();
    while (invalid_command) {
        invalid_command = false;
        pc.printf("Welcome! Please enter a command:\n\r");
        pc.printf("> ");
    
        char char_buf[MAX_LINE];
        int buf_length = 0;
        
        char c = pc.getc();
        while (c!='\n' && c!='\r' && buf_length < MAX_LINE - 1) {
            char_buf[buf_length++] = c;
            pc.printf("%c", c);
            c = pc.getc();
        }
        char_buf[buf_length] = '\0';
        pc.printf("\n\r");
        pc.printf("Parsing command string '%s'...\n\r", char_buf);
        
        char command1;
        char command2;
        char noteString[20];
        float revs = 0;
        float vel = 0;
        if (sscanf(char_buf, "%c%f%c%f", &command1, &revs, &command2, &vel)==4 && command1=='R' && command2=='V') {
            pc.printf("  Received R-V command with\n\r");
            pc.printf("  Revolutions: %3.2f\n\r", revs);
            pc.printf("  Velocity: %3.3f\n\r", vel);
            targetRev = revs;
            targetVel = vel;
        } else if (sscanf(char_buf, "%c%f", &command1, &revs) == 2 && command1=='R') {
            pc.printf("  Received R command with\n\r");
            pc.printf("  Revolutions: %3.2f\n\r", revs);
            targetRev = revs;
            targetVel = 0;
        } else if (sscanf(char_buf, "%c%f", &command1, &vel) == 2 && command1=='V') {
            pc.printf("  Received V command with\n\r");
            pc.printf("  Velocity: %3.3f\n\r", vel);
            targetRev = 0;
            targetVel = vel;
        } else if (sscanf(char_buf, "%c%[^\t\n\r]", &command1, &noteString) == 2 && command1=='T') {
            pc.printf("  Received T command with\n\r");
            pc.printf("  Notes: %s\n\r", noteString);
            char note[2];
            int duration;
            int index = 0;
            while (sscanf(noteString, "%[^0123456789]%d", &note, &duration) == 2) {
                pc.printf("Note: %s \tDuration: %d\n\r", note, duration);
                notesList[index] = (int)note[0];
                durationsList[index] = duration;
                index++;
                for(int i = 2; i < 20; i+=1) {
                    noteString[i-2] = noteString[i];
                }
            }
            notesList[index] = -1;
            durationsList[index] = -1;
            
            targetRev = 0;
            targetVel = 0;
        } else {
            pc.printf("  ERR: Invalid command\n\r", char_buf);
            invalid_command = true;
        }
    }
    stdio_mutex.unlock();
    
    regex_queue.put((uint32_t*) 1); // Send message to indicate that regex_done    

    updateAngle();
    regex_thread.terminate();
}

//------------------------------------------------------------------------------------------


//Main
int main() {
    int8_t orState = 0; // Rotor offset at motor state 0    
    //Initialise the serial port
    Serial pc(SERIAL_TX, SERIAL_RX);
    int8_t intState = 0;
    int8_t intStateOld = 0;
    
    stdio_mutex.lock();
    pc.printf("Hello\n\r");
    stdio_mutex.unlock();
    
    //Run the motor synchronisation
    orState = motorHome();
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //---------------------------------------- initiate the threads
    regex_thread.start(&regex);
    //-------------------------------------------------------------
    osEvent evt = regex_queue.get();
    regex_done = evt.value.v;
    
    //-----------------------------------------initiate the ISRs
    channelA.rise(&channelA_rise);
    channelA.fall(&channelA_fall);
    channelB.rise(&channelB_rise);
    channelB.fall(&channelB_fall);
    photoI1.rise(&angle_60);
    photoI2.rise(&angle_60);
    photoI3.rise(&angle_60);
    photoI1.fall(&angle_60);
    photoI2.fall(&angle_60);
    photoI3.fall(&angle_60);
    //----------------------------------------------------------
    control_thread.start(&controllerFunction);

    stdio_mutex.lock();
    pc.printf("Control thread started!\n\r");
    stdio_mutex.unlock();
            
    L1L.period(period);
    L1H.period(period);
    L2L.period(period);
    L2H.period(period);
    L3L.period(period);
    L3H.period(period);
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    int printCounter = 0;
    while (1) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6, dutyCycle); //+6 to make sure the remainder is positive
        }
        if ((printCounter == 100000) && (regex_done)) {
            stdio_mutex.lock();
            pc.printf("Rev=%.2f \tVel=%.2f \tDutyCycle=%.5f \tPhaseLead=%d \t targetRev=%.2f \ttargetVel=%.2f\n\r", live_angle/360, live_velocity, dutyCycle, lead, targetRev, targetVel);
            stdio_mutex.unlock();
            printCounter = 0;
        }
        printCounter++;
    }
}
