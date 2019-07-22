// Laser Shutter Controller for harddrive-coil laser shutters
// 
// This 'fast' shutter controller accellerates and brakes the shutter motion
// before going into a lower power standby mode (recharging the capacitors).
// Accelleration / brake times need to be optimized for each shutter / power 
// supply combination! (see below)
//
// Once a program has been saved to EEPROM, LoadSettings() can be called in the 
// setup routine to have persistence of a programmed motion routine.
//
// Shutters can be controlled via USB communication with the Arduino (19200 baud),
// or toggled with a pull-down pushbutton. The shutter can be programmed to open and
// close for predefined times (single or continuous operation).
// Maybe add functions to control a second shutter in the future,
// similar to the 4 shutter box we operate in the CRASY experiment.
// (Use OCR2B on pin D3 for second PWM, two more digital pins for direction.)
//
// Feel free to use, redistribute and/or modify this program under the terms of the 
// MIT License (<https://opensource.org/licenses/MIT>). 
// I offer no warranty or support.
//
// // Thomas Schultz
//
// Comments on setting the acceleration / breaking values:
// 'set ao 20000' sets the opening acceleration to 20 ms. The shutter accellerates full power.
// 'set bo 100-0' sets teh rbeaking time to 10 ms. The shutter breaks with full power.
// 'set op 50' sets the holding power to 50/255 of the max power. 
// The close commands (ac, bc, cp) work the same way. 
/*** The shutters are controlled by an H-bridge driver module (e.g., L298N, or TB6612FNG). 
 *  
 *  Description for L298N module <http://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/>
 *  Two digital pins (connected to In1,2) control the direction of 
 *  the shutter current; e.g., 1,0 to open and 0,1 to close the shutter. 
 *  A PWM pin (connected to ENA) controls the power. Full power allows fast 
 *  opening/closing and reduced holding power keeps everything cool.
 *  Fast PWM uses timer 2 to avoid interference with delay functions.
 *  Pins 3 (OC2B) and 11 (OC2A) are controlled by timer 2,m I use OC2A.
 *  Fast PWM (WGM22,1,0 = 0,1,1) --> set OC2A (pin 11) at BOTTOM (0), 
 *  clear at OC2A==OCR2A, run to TOP (255). OCR2A thereby controls the high/low 
 *  ratio on pin11, e.g., OCR2A=25 is about 10% high for 10% shutter current.
 *  (cf. ATMEL328 datasheet to find TCCR2A and TCCR2B bits for fast PWM)
 *  
 *  Run PWM at highest frequency (f_clk/256) to avoid noise from the shutter coil.
 *  
 *  Setup: Connect the L298N module In1, In2 and ENA pins to the #defined digital 
 *  pins of the Arduino. Connect a pushbutton to the #defined digital pin of the 
 *  Arduino. Connect 5V and ground to Arduino pins. 
 *  Connect a suitable power supply to the L298 module (I use a 15V supply to power 
 *  the Arduino and the L298 module). Bigger PS -> faster motion.
 *  Upload and run the program.
 **************************************************************************  
 *  Description for TB6612FNG module (https://www.sparkfun.com/products/14451)
 *  
 *  AIN1 and AIN2 control the motion direction (connect to pinA1, pinA2).
 *  STBY pull high.
 *  PWMA controls speed (connect to powerPinA)
 *  VM to power supply (12V)
 *  VCC to Arduino 5V power
 *  GND to Ground
 *  Maximum PWM frequency is 100 kHz, so 62.5 kHz with 8-bit timer2 running at full 16MHz will work.
 **************************************************************************/

#include <EEPROM_anything.h>              // EEPROM library to save / load complex data
                                          // (http://playground.arduino.cc/Code/EEPROMWriteAnything)
#include <SerialCommand_TS.h>             // Command library to handle serial commands
                                          // This is identical to kroimon/Arduino-SerialCommand but I like to 
                                          // ignore the case of commands; line 75: "char inChar = tolower(Serial.read());"
#define ProgString "Shutter Controller"
#define EEPROM_ID -29999                  // Arbitrary number to determine if EEPROM has been initialized
/************* Shutter Pin connections ***************/
#define EnablePin 8                                 // Enable pin (for TB6612FNG)
#define pinA1 9                                     // Shutter open pin, connect to In1
#define pinA2 10                                    // Shutter close pin, connnect to In1
#define powerPinA 11                                // Must be 11 or 3 for timer 2 PWM, connect to ENA
#define CloseA digitalLow(pinA2);digitalHigh(pinA1) // Shutter A, pinA1,pinA2 = 1,0 to open
#define OpenA digitalLow(pinA1);digitalHigh(pinA2)  // Shutter A, pinA1,pinA2 = 0,1 to close
#define PushButtonPin 12                            // Digital pin for manual shutter button

/************ Other defines *************/
#define _EEPROM_save_position 0           // EEPROM position to save settings
#define DebounceDelay 50                  // Debounce delay in milliseconds (the big colored buttons bounce << 1 ms)
#define LEDPin 13                         // Pin for LED feedback on shutter A

// --- Recognize Type of Arduino ---//
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) 
const String ID = "Arduino UNO";
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) 
const String ID = "Arduino Leonardo";
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
const String ID = "Arduino Mega";
#else
const String ID = "Arduino";
#endif

//--- Fast digital read/write ---//
#define portOfPin(P)  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)   (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)   (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)   ((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)    ((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)      // Set pin low
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)      // Set pin high
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)       // True for high pin
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)       // True for low pin
#define digitalState(P)((uint8_t)isHigh(P))             // Read pin (1/0)

SerialCommand sCmd;                       // Serial command object for serial user input

/************ Timing constants for shutter open/close ***************/
// Must be optimized for each shutter, power supply pair! Use 'set' commands //
// Full power accelerate (us), then break (us), then hold with lower power. //
/* Values below for Samsung HD Shutter with 12 V, 2.5A supply, TB driver */
unsigned _accelClose = 8100;    // Accelleration time (us) to close; Gravity + spring help motion
unsigned _breakClose = 5900;    // Break time (us)
int _closePower = 0;            // Holding power (PWM; /255), spring + gravity is enough.
unsigned _accelOpen = 9300 ;    // Accelleration time (us) to open; Gravity + spring resist motion
unsigned _breakOpen = 5100 ;    // Break time (us)
int _openPower = 40   ;         // Holding power against gravity and spring (PWM; /255)

/************ Other variables ***************/
int _Verbose = 1;                         // Explicit answers over Serial
boolean OpenValue = 1;                    // Digital values corresponding to open position (menu option to reassign open values)
boolean LEDOpenValue = 1;                 // LED output for open shutter
int progOpenTime = 10;                    // Programmable open time in ms
int progPeriod = 1000;                    // Programmable motion period in ms
//bool a = false;
bool progRunning = false;                 // Program running?
long progTime;
char buf[17];                             // Buffer for text output

/* -------- Setup -------- */
void setup(){
  Serial.begin (19200);                    // Set up the Serial Connection
  Help();                                  // Print command list to serial

  pinMode(pinA1, OUTPUT);                  // Pin to open shutter A
  pinMode(pinA2, OUTPUT);                  // Pin to close Shutter A
  pinMode(powerPinA, OUTPUT);              // PWM pin to control power for shutter A
  pinMode(LEDPin, OUTPUT);                 // Pin for indicator LED
  pinMode(PushButtonPin, INPUT_PULLUP);    // Pull pushbutton pin high
  pinMode(EnablePin, OUTPUT);              // Need to enable current
  digitalHigh(EnablePin);

  TCCR2A = B10100011;                      // Set up timer2 for fast PWM, 
  TCCR2B = B00000001;                      // Full speed timer (16MHz/256) to avoid noise
                                           // (Should emit 62.5 kHz noise. )
  OCR2A = 25;                              // 25 is approx. 10% power
  _close();                                // Start with closed shutter;
                                           // Consider restoring saved shutter positions or default positions here  
//--- My Serial Commands are case insensitive, use only lower-case command strings ---//
  sCmd.addCommand("q", QueryPositions);    // Print byte of shutter positions
  sCmd.addCommand("o", Open);              // Open one or all shutters
  sCmd.addCommand("c", Close);             // Close one or all shutters
  sCmd.addCommand("t", Toggle);            // Toggle one or all shutters
  sCmd.addCommand("prog", Program);        // Change shutter program
  sCmd.addCommand("p", ProgramStart);      // Start program
  sCmd.addCommand("set", Settings);        // Change shutter settings
  sCmd.addCommand("save", SaveSettings);   // Save program to EEPROM (load in setup)
//  sCmd.addCommand("load", LoadSettings);   // Load shutter positions from EEPROM; only for testing
  sCmd.addCommand("verbose", Verbose);     // Toggle verbose answers
  sCmd.addCommand("help", Help);           // Print command list
  sCmd.addCommand("id", IDN);              // Print ID
  sCmd.setDefaultHandler(unrecognized);    // Handler for unexpected commands (says "What?")

  LoadSettings();                          // Load saved settings from EEPROM
//  Settings();                            // Inform user about loaded settings
}


/* -------- Program Loop -------- */
void loop(){
  sCmd.readSerial();                            // Process serial commands
  int pb = digitalRead(PushButtonPin);          // Read push-button state
  if (isLow(PushButtonPin)) _debounceToggle();  // ... toggle shutter after button release
  if (progRunning) {
    if (millis() > progTime) ProgramExecute();  // Run user program (no protection against timer overflow)
  }
}

/* ----- UTILITY FUNCTIONS ----- */

void _debounceToggle(){       //--- TOGGLE SHUTTER AFTER BUTTON PUSHED---//
  delay(DebounceDelay);                       // Wait debounce delay 
  while(!digitalRead(PushButtonPin)){}        // Wait for pushbutton release
  _toggle();                                  // Toggle shutter position
  progRunning = 0;                            // Stop running program
  delay(DebounceDelay);                       // Wait debounce delay 
}

void _toggle(){              //--- TOGGLE SHUTTER ---//
  if (isHigh(LEDPin) == LEDOpenValue) _close(); // ... close shutter
  else _open();                                 // ... or open shutter
}

void _open(){               //--- OPEN SHUTTER ---//
  if (isHigh(LEDPin) == LEDOpenValue) return;  // Shutter is already open
  OCR2A = 255;                                 // Full power
  OpenA;                                       // Open shutter
  if (_accelOpen < 16383) delayMicroseconds(_accelOpen);    // ... wait
  else delay(_accelOpen/1000);
  if (_breakOpen) CloseA;                      // Break the shutter motion
  delayMicroseconds(_breakOpen);               // ... wait
  OpenA;                                       // Hold shutter open
  OCR2A = _openPower;                          // Reduced power to hold position
  if(_Verbose) Serial.println("opened shutter");
  digitalWrite(LEDPin, LEDOpenValue); 
}

void _close(){            //--- CLOSE SHUTTER ---//
  if (isHigh(LEDPin) == !LEDOpenValue) return; // Shutter is already closed
  OCR2A = 255;                                  // Full power
  CloseA;                                       // Close shutter
  if (_accelClose< 16383) delayMicroseconds(_accelClose);              // ... wait
  else delay(_accelClose/1000);
  if (_breakClose) OpenA;                     // Break the shutter motion
  delayMicroseconds(_breakClose);              // ... wait
  CloseA;                                       // Hold shutter open
  OCR2A = _closePower;                         // Reduce power to hold position
  if(_Verbose) Serial.println("closed  shutter");
  digitalWrite(LEDPin, !LEDOpenValue); 
}

void ProgramExecute(){      // --- Perform  programmed opening of shutter --- //
  _open();
  delay(progOpenTime);                  // (open and close once)
  _close();
  if (!progPeriod) progRunning = false; // Stop after single run.
  else progTime += progPeriod;          // Or set time for next execution
}

/* --- USER CALLABLE COMMANDS --- */

void Open(){ _open(); }      //--- OPEN SHUTTER ---//
void Close(){  _close(); }   //--- CLOSE SHUTTER ---//
void Toggle(){ _toggle(); }  //--- TOGGLE SHUTTER ---//

void QueryPositions(){       //--- LIST SHUTTER POSITIONS ('q') ---//
  if (digitalRead(LEDPin) == LEDOpenValue)
    {Serial.println("shutter open");}             // Print shutter status
  else Serial.println("shutter closed");
}

void SaveSettings(){        //--- SAVE PROGRAM & SETTINGS TO EEPROM ('save') ---//
  cli();                                                            // Stop interrupts for EEPROM read/write
  int pos = _EEPROM_save_position;
  int _EEPROM_init; 
  EEPROM_readAnything(pos, _EEPROM_init); pos+=2;                   // Read if EEPROM is initialized
  if (_EEPROM_init != EEPROM_ID){
    EEPROM_writeAnything(pos-2, EEPROM_ID);                         // First initialization of EEPROM with EEPROM_ID                      
  }
  EEPROM_writeAnything(pos, _Verbose); pos+=2;                      // Write Verbose settings
  EEPROM_writeAnything(pos, _accelClose); pos+=2;                   // Write close acceleration time  
  EEPROM_writeAnything(pos, _breakClose); pos+=2;                   // Write close break time 
  EEPROM_writeAnything(pos, _closePower); pos+=2;                   // Write close holding power  
  EEPROM_writeAnything(pos, _accelOpen); pos+=2;                    // Write open acceleration time
  EEPROM_writeAnything(pos, _breakOpen); pos+=2;                    // Write open break time
  EEPROM_writeAnything(pos, _openPower); pos+=2;                    // Write open holding power
//  EEPROM_writeAnything(pos, OpenValue); pos+=2;                   // Write LED value for open shutter
//  EEPROM_writeAnything(pos, LEDOpenValue); pos+=2;                // Write LED value for open shutter
  EEPROM_writeAnything(pos, progOpenTime); pos+=2;                  // Write D0-D7 values (too lazy to go through the pin mapping here)
  EEPROM_writeAnything(pos, progPeriod); pos+=2;                    // Write D8-D13 values
  EEPROM_writeAnything(pos, progRunning); pos+=2;                   // Write D8-D13 values
  sei();                                                            // Activate interrupts
  if(_Verbose) Serial.println("Settings saved to EEPROM");
}

void LoadSettings(){       //--- READ PROGRAM & SETTINGS FROM EEPROM ('load') ---//
// This routine does not check if saved values exist! //
  cli();                                                            // Stop interrupts for EEPROM read/write
  int pos = _EEPROM_save_position;
  int _EEPROM_init; 
  EEPROM_readAnything(pos, _EEPROM_init); pos+=2;                   // Read if EEPROM is initialized
  if (_EEPROM_init != EEPROM_ID) {
    Serial.println("Type 'save' to initialize settings");           // EEPROM not initialized
  }
  else {
    EEPROM_readAnything(pos, _Verbose); pos+=2;                     // Read Verbose settings
    EEPROM_readAnything(pos, _accelClose); pos+=2;                  // Read close acceleration time  
    EEPROM_readAnything(pos, _breakClose); pos+=2;                  // Read close break time 
    EEPROM_readAnything(pos, _closePower); pos+=2;                  // Read close holding power  
    EEPROM_readAnything(pos, _accelOpen); pos+=2;                   // Read open acceleration time
    EEPROM_readAnything(pos, _breakOpen); pos+=2;                   // Read open break time
    EEPROM_readAnything(pos, _openPower); pos+=2;                   // Read open holding power
//  EEPROM_readAnything(pos, OpenValue); pos+=2;                    // Read LED value for open shutter
//  EEPROM_readAnything(pos, LEDOpenValue); pos+=2;                 // Read LED value for open shutter
    EEPROM_readAnything(pos, progOpenTime); pos+=2;                 // Read program open time
    EEPROM_readAnything(pos, progPeriod); pos+=2;                   // Read program period
    EEPROM_readAnything(pos, progRunning); pos+=2;                  // Read whether program should be running
  }
  sei();                                                            // Activate interrupts
  if(_Verbose) Serial.println("Settings loaded from EEPROM");
}

void Verbose(){              // --- TOGGLE EXPLICIT ANSWERS ('verbose') --- //
 char *arg;
  arg = sCmd.next();                         // Read number string
  if (arg != NULL) {                         // Did the user give a number?
    _Verbose = atoi(arg);}                   // Convert number string to long integer
  if(_Verbose) Serial.print("Verbose: ");    // Print current value for Verbose
  Serial.println(_Verbose); 
}

void IDN() {                // --- QUERY DEVICE IDENTIFICATION ('id') --- //
  Serial.print("ID: ");
  Serial.print(ID);
  Serial.print(" + ");
  Serial.println(ProgString);
}

void ProgramStart(){        // --- Start program ('p') --- //
  progTime = millis();            // Time for next program execution (now)
  progRunning = !progRunning;     // Toggle program run state
}

void Program(){             // --- Create program for opening of shutter ('prog *char *int') --- //
  char *arg;
  if(Verbose) Serial.println("Program");
  arg = sCmd.next();                         // Read next command string
  if (arg != NULL) {                         // Did the user give a value?
    if (strncmp(arg, "o", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Read open time
      if (arg != NULL) progOpenTime = atoi(arg);
      else Serial.println("Please give open time in ms");
    }
    if (strncmp(arg, "p", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Read program period
      if (arg != NULL) progPeriod = atoi(arg);
      else Serial.println("Please give program period in ms");
    }
  }
  else {
    Serial.print("Program period: ");
    Serial.println(progPeriod);
    Serial.print("Programmed opening time: ");
    Serial.println(progOpenTime);
    Serial.print("Running: ");
    Serial.println(progRunning);
    }
}

void Settings(){           // --- Set shutter acceleration and holding parameters ('set *char *int') --- //
  char *arg;
  if(Verbose) Serial.println("Settings");
  arg = sCmd.next();                         // Read next command string
  if (arg != NULL) {                         // Did the user give a value?
    if (strncmp(arg, "ac", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _accelClose = atoi(arg);
    }
    if (strncmp(arg, "bc", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _breakClose = atoi(arg);
    }
    if (strncmp(arg, "cp", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _closePower = atoi(arg);
    }
        if (strncmp(arg, "ao", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _accelOpen = atoi(arg);
    }
        if (strncmp(arg, "bo", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _breakOpen = atoi(arg);
    }
        if (strncmp(arg, "op", SERIALCOMMAND_MAXCOMMANDLENGTH) == 0){ 
      arg = sCmd.next();                     // Closing accelleration time
      if (arg != NULL) _openPower = atoi(arg);
    }
  }
  Serial.print("accelClose = "); Serial.println(_accelClose);
  Serial.print("breakClose = "); Serial.println(_breakClose);
  Serial.print("closePower = "); Serial.println(_closePower);
  Serial.print("accelOpen  = "); Serial.println(_accelOpen);
  Serial.print("breakOpen = "); Serial.println(_breakOpen);
  Serial.print("openPower = "); Serial.println(_openPower);
}

void Help() {             // --- PRINT LIST OF COMMANDS ('help') --- //
  IDN();
  Serial.println();
  Serial.println("Command list (* indicates argument type)");    // Instructions
  Serial.println("o:    Open shutters");
  Serial.println("c:    Close shutter");
  Serial.println("t:    Toggle shutter");
  Serial.println("q:    Query shutter position");
  Serial.println("prog: Program shutter");
  Serial.println("      p *int: Period (ms); 0 for single motion");
  Serial.println("      o *int: Open time (>10 ms)");
  Serial.println("p:    Start/Stop program");
  Serial.println("      ao *int: accelOpen time (us)");
  Serial.println("      bo *int: breakOpen time (us)");
  Serial.println("      op *int: openPower (0-255)");
  Serial.println("      sc,bc,cp: ... settings for close");
  Serial.println("set:  Settings (see README)");
//--- Commands below are only interesting for multiple shutters ---//
//  Serial.println("      (missing argument affects all shutters)");
//  Serial.println("q: *  Query delays in byte format");
//  Serial.println("o:    Open shutters *int or all");
//  Serial.println("c:    Close shutter *int or all");
//  Serial.println("t:    Toggle shutter *int or all");
//  Serial.println("verbose: Toggle verbose answers");
//  Serial.println("set *str *int: Change settings; ");
//  Serial.println("  o *i:        Set open time in microseconds; ");
//  Serial.println("  h *i:        Set hold current (%); ");
  Serial.println("save: Save program");
//  Serial.println("load: Load program");
  Serial.println("help: Print command list"); 
  Serial.println("id:   Print device ID"); 
  
  Serial.println();
}

void unrecognized(const char *command) {    // --- HANDLE NONEXISTING COMMAND --- //
  Serial.print("I don't understand the command '");
  Serial.print(command);
  Serial.println("'");
}
