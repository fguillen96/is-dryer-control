// *******************************************************************************************************
// DESCRIPTION: MPPT and Drive control 
// AUTOR: Fran Guillén
// BOARD: M-Duino 50RRE
// VERSION: Using delay

//********************************************************************************************************

#include <Arduino.h>


// ********************************************************************
//                     PIN CONFIGURATION
// ********************************************************************
// ---------- ANALOG INPUTS ---------
#define AI_VDC  I0_2    // Analog INPUT I0.2 --> DC Voltage
#define AI_G    I0_3    // Analog INPUT I0.3 --> Irradiance
#define AI_T    I0_4    // Analog INPUT I0.4 --> Temperature
#define AI_F    I0_5    // Analog INPUT I0.5 --> Frequency


// ---------- DIGITAL INPUTS ---------
#define DI_ALARM            I1_2      // Digital INPUT I0.1 --> Alarm signal
#define DI_DRIVE_0HZ        I1_3      // Digital INPUT I0.0 --> HIGH if 0 Hz
#define DI_RUN_CONTROL      I1_4      // Digital INPUT I1.0 --> Run signal

// ---------- DIGITAL OUTPUTS ---------
#define DO_RUN              R0_1      // Digital OUTPUT RELAY R0.1 --> Signal to start or stop frequency drive
#define DO_RUN_LED          R0_2      // Digital OUTPUT RELAY R0.2 --> Signal to control run led status
#define DO_ALARM_LED        R0_3      // Digital OUTPUT RELAY R0.3 --> Signal to control alarm led status
#define DO_RESET            R0_4      // Digital OUTPUT RELAY R0.4 --> Signal reset drive

// ---------- ANALOG OUTPUT ----------
#define AO_SETPOINT         A2_5      // Analog OUTPUT A2.5 (0-10 V) -->  Vmpp Setpoint




// ********************************************************************
//                     SYSTEM PARAMETERS
// ********************************************************************
const int kNumberCellsModule = 36;              // Number of cells per module
const int kNumberModulesSeries = 18;            // Number of modules in series
const int kPmpp = 50;                           // Maximun power point of module (W)
const int kVmppMin = 260;                       // Minium power point voltage (V)
const int kPMinToOff = 50;                      // Minimum value of DC power to stop drive (W) 
const int kPThldToOn = 60;                      // DC power value trehsold to start drive (W) 
const int kVdcMinMot = 260;                     // Minimum DC voltage with maximum power pump LUISMI

const float kVmppModule = 18.3;                 // Maximum power point voltage of module (V)
const float kDVocDt = 0.067;                    // Voc variation with temperature (V/ºC)
const float kPmppDt = 0.19;                     // Pmpp variation with temperature (W/ºC)
const float kFmaxOut = 130;                     // Max output frequency (Hz)



// ********************************************************************
//                        GLOBAL VAR
// ********************************************************************
enum States {stopped, stopping, OnMptt, Alarm};
int state = 1;



// ********************************************************************
//                     FUNCTION PROTOTYPES
// ********************************************************************
int CalculateMpp(float, float);
int SecondsOverIrradianceThld(int, int);
int SecondsUnderIrradianceThld(int, int);


// ********************************************************************
//                           BOARD SETUP
// ********************************************************************
void setup() {
  // ----- DIGITAL OUTPUTS -----
  pinMode(DO_RUN, OUTPUT);
  pinMode(DO_RUN_LED, OUTPUT);
  pinMode(DO_ALARM_LED, OUTPUT);
  pinMode(DO_RESET, OUTPUT);

  // ----- DIGITAL INPUTS -----
  pinMode(DI_ALARM, INPUT);
  pinMode(DI_DRIVE_0HZ, INPUT);
  pinMode(DI_RUN_CONTROL, INPUT);
  //digitalWrite(DI_RUN_CONTROL, HIGH);

  // ----- ANALOG OUTPUTS -----
  pinMode(AO_SETPOINT, OUTPUT);

  // ----- SERIAL PORT -----
  Serial.begin(9600);
}

void loop() {
  // --------- READING ANALOG INPUTS ---------
  // Sensor max output is 5V (except Vdc sensor), max adc output is 511
  int dc_voltage = analogRead(AI_VDC) * 1000.0 / 1023.0;          // 10V sensor --> 1000Vdc
  int irradiance = analogRead(AI_G) * 1164.0 / 511.0;             // 5V sensor --> 1164w/cm2
  float cell_temp = analogRead(AI_T) * 150.0 / 511.0 - 20.0;      // 0V sensor --> -20ºC and 5V sensor --> 130ºC
  float frequency = analogRead(AI_F) * 50.0 / 511.0;              // 5  sensor --> 50Hz

  // ---------- READING DIGITAL INPUTS ---------
  bool alarm = digitalRead(DI_ALARM);                     // Alarm signal. Could be interrupt...
  bool run_signal_on = digitalRead(DI_RUN_CONTROL);
  bool drive_0hz = digitalRead(DI_DRIVE_0HZ);

  // ---------- CALCULATING THRESHOLD ----------
  bool run_allowed = false;
  if (SecondsOverIrradianceThld(irradiance, kPThldToOn, cell_temp) > 60) run_allowed = true;

  bool stop_needed = false;
  if (SecondsUnderIrradianceThld(irradiance, kPMinToOff, cell_temp) > 60) stop_needed = true;

  // ---------- CALCULATING MPP BASED ON CELL TEMP AND FREQUENCY----------
  int vd_mpp = CalculateMpp(cell_temp, frequency);
  //TODO: ¿Qué conversion hay que hacer?
  int write_setpoint = map(vd_mpp, 0, 1000, 0, 255);
  analogWrite(AO_SETPOINT, write_setpoint);
  


// ---------- STATE MACHINE ----------
  switch (state) {
    case stopping: {
      // Send power off signal to drive
      digitalWrite(DO_RUN, LOW);

      // Switch off run led
      digitalWrite(DO_RUN_LED, LOW);  
      
      // When pump is stopped, go to stopped status
      if (drive_0hz) state = stopped;

      if (alarm) state = Alarm;
    }
    break;

    case stopped: {
      // Turn off run led
      digitalWrite(DO_RUN_LED, LOW);    
      
      if (run_signal_on && run_allowed) {
        state = OnMptt;
        delay(200);
      }
      if (alarm) state = Alarm;
    }
    break;

    case OnMptt: {
      // Turn on RUN signal to drive
      digitalWrite(DO_RUN, HIGH);

      // Turn on RUN led
      digitalWrite(DO_RUN_LED, HIGH);

      if (!run_signal_on || stop_needed) {
        state = stopping;
        delay(200);
        //TODO: HIGH OR LOW???
      }

      if (alarm) state = Alarm;
    }
    break;

    case Alarm: {
      static int n_blinks = 0;
      digitalWrite(DO_ALARM_LED, digitalRead(DO_ALARM_LED) ^ 1 );
      delay(500);      

      //TODO: Sacar señal de reset
      if (n_blinks >= 120) {
        digitalWrite(DO_RESET, HIGH);
        delay(100);
        digitalWrite(DO_RESET, LOW);
        n_blinks = 0;
        state = stopped;
      }
    }
    break;
  }

  static unsigned long prev = 0;
  if (millis() - prev >= 500)
  {
    prev = millis();
    Serial.print("Alarm: ");
    Serial.println(alarm);
    Serial.print("Run signal: ");
    Serial.println(run_signal_on);
    Serial.print("Stopped: ");
    Serial.println(drive_0hz);
    Serial.print("State: ");
    Serial.println(state);
    Serial.print("G: ");
    Serial.println(irradiance);
    Serial.print("Tc: ");
    Serial.println(cell_temp);
    Serial.print("DC voltage: ");
    Serial.println(dc_voltage);
    Serial.print("Frequency: ");
    Serial.println(frequency);
  }


}


int CalculateMpp(float cell_temp, float frequency_out) {
  int vdc_sp = (cell_temp - 25) *  kNumberModulesSeries * kNumberCellsModule * kDVocDt;

  vdc_sp = ((kNumberModulesSeries * kVmppModule) - vdc_sp);

  // If setpoint is smaller than minimun setpoint...
  if (vdc_sp < kVmppMin) vdc_sp = kVmppMin;
  
  // Calculate some this voltage...
  int vdc_sp_duty_100 = frequency_out * kVdcMinMot;

  // If setpoint is smaller than previous calculated setpoint...
  if(vdc_sp < vdc_sp_duty_100) vdc_sp = vdc_sp_duty_100;

  // 0-1000V --> 0-10V in analog input of frequency drive
  vdc_sp = vdc_sp / 100; 

  return vdc_sp;
}


int SecondsOverIrradianceThld(int irradiance, int threshold, float cell_temp) {
  static int seconds_over = 0;
  static unsigned long pre_millis = millis();
  float Pdc = irradiance / 1000 * kPmpp * (1-(cell_temp - 25) * kPmppDt);
  if (Pdc > threshold) {
    seconds_over += millis() - pre_millis;
  }
  else seconds_over = 0;

  pre_millis = millis();

  return seconds_over / 1000;
}

int SecondsUnderIrradianceThld(int irradiance, int threshold, float cell_temp) {
  static int seconds_under = 0;
  static unsigned long pre_millis = millis();
  float Pdc = irradiance / 1000 * kPmpp * (1-(cell_temp - 25) * kPmppDt);
  if (Pdc < threshold) {
    seconds_under += millis() - pre_millis;
  }
  else seconds_under = 0;

  pre_millis = millis();

  return seconds_under / 1000;
}