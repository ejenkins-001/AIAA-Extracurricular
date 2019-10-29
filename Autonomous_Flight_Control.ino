
//**                                                This is our Autonomous Flight Control System                                                **//

//** The Adafruit_LSM9DS0 sensor is used in this program to get the orientation data of the aircraft                                            **//
//** This orientation data comes from the gyroscope and accelerometer sensors onboard the LSM9DS0                                               **//
//** This code implements a complementary filter which combines the gyroscope and accelerometer data to give us the orientation of the aircraft **//

//** After the complementary filter returns the orientation data, we check to see if the orientation has changed                                **//
//** If the orientaition has changed then we write a command to the servo which will counteract that change in rotation                         **//
//** Then the programs loops around to get a new orientation value and the process repeats                                                      **//


//**                                                This is our Manual Flight Control System                                                                      **//

//** The purpose of this program is to pass the transmitter signal directly to the servos that control the aircraft thus creating manual control of the airplane **//

//** The Transmitter signal is sent to the Receiver aboard the airplane. The Receiver then produces a pulselength output that would normally control the servos  **//
//** Since the Arduino is needed as an interface, the Receiver pulselength output is directly read by the Arduino.                                               **//
//** The Arduino then passes this pulselength directly to the servos                                                                                             **//                                                                  **//
//** Thus the Arduino acts as an intermediater between the Receiver and the servos                                                                               **//

//** The Arduino is needed as an interface so that we can switch between manual control inputs comming from the Transmitter/Receiver systems                     **//
//** And the Autonomous control inputs coming from the LS9DS0 and Arudino system                                                                                 **//

#include <SPI.h>                  
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_PWMServoDriver.h>
#include <PinChangeInt.h>

#include <EEPROM.h>                     //Manual

/*Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground
   Connect servo to pin 9*/

  Servo aileron_servo;                
  uint8_t servonum_0 = 0;
  uint8_t servonum_1 = 1;
  uint8_t servonum_2 = 2;
  uint8_t servonum_3 = 3;
  uint8_t servonum_4 = 4;
  uint8_t servonum_5 = 5;
  
  
  int degrees;
  int pulselength;
  int prevPulselength;
  int GEAR;
  

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 150 to 550
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // this is the 'maximum' pulse length count (out of 4096)

// Assign your channel in pins
#define THROTTLE_IN_PIN 3
#define AILERON_IN_PIN 4
#define ELEVATOR_IN_PIN 5
#define RUDDER_IN_PIN 6
#define GEAR_IN_PIN 7
#define AUX_IN_PIN 8

// Assign your channel out pins
//#define THROTTLE_OUT_PIN 5
//#define AILERON_OUT_PIN 9
//#define GEAR_OUT_PIN 10

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoThrottle;
Servo servoAileron;
Servo servoElevator;
Servo servoRudder;
Servo servoGear;
Servo servoAux;


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 3
#define AILERON_FLAG 4
#define ELEVATOR_FLAG 5
#define RUDDER_FLAG 6
#define GEAR_FLAG 7
#define AUX_FLAG 8

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unAileronInShared;
volatile uint16_t unElevatorInShared;
volatile uint16_t unRudderInShared;
volatile uint16_t unGearInShared;
volatile uint16_t unAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulAileronStart;
uint32_t ulElevatorStart;
uint32_t ulRudderStart;
uint32_t ulGearStart;
uint32_t ulAuxStart;


/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
     Set last read angle function
*/
/**************************************************************************/
// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) 
{
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

/**************************************************************************/
/*
     Calibrage sensor function
*/
/**************************************************************************/
// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
void calibrate_sensors() 
{
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  
  //Serial.println("Starting Calibration");

  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) 
  {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    x_accel += accel.acceleration.x;
    y_accel += accel.acceleration.y;
    z_accel += accel.acceleration.z;
    x_gyro +=  gyro.gyro.x;
    y_gyro +=  gyro.gyro.y;
    z_gyro +=  gyro.gyro.z;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
//while (!Serial);  // wait for flora/leonardo
  
  Serial.begin(9600);  
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
  
 // aileron_servo.attach(9); /* Servo output at pin 9 */  // AUTONOMOUS
  
  //servoThrottle.attach(THROTTLE_OUT_PIN);
  //servoAileron.attach(AILERON_OUT_PIN);
  //servoGear.attach(GEAR_OUT_PIN);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE); 
  PCintPort::attachInterrupt(AILERON_IN_PIN, calcAileron,CHANGE); 
  PCintPort::attachInterrupt(ELEVATOR_IN_PIN, calcElevator,CHANGE);
  PCintPort::attachInterrupt(RUDDER_IN_PIN, calcRudder,CHANGE);
  PCintPort::attachInterrupt(GEAR_IN_PIN, calcGear,CHANGE); 
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);
  
  pwm.begin();
  pwm.setPWMFreq(60);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{  
   GEAR = pulseIn(7, HIGH, 20000);

//MANUAL
           while( GEAR <= 1500 )
            {
              
              // create local variables to hold a local copies of the channel inputs
              // these are declared static so that thier values will be retained 
              // between calls to loop.
              static uint16_t unThrottleIn;
              static uint16_t unAileronIn;
              static uint16_t unElevatorIn;
              static uint16_t unRudderIn;
              static uint16_t unGearIn;
              static uint16_t unAuxIn;
              // local copy of update flags
              static uint8_t bUpdateFlags;

              // check shared update flags to see if any channels have a new signal
              if(bUpdateFlagsShared)
              {
                noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

                // take a local copy of which channels were updated in case we need to use this in the rest of loop
                bUpdateFlags = bUpdateFlagsShared;
    
                // in the current code, the shared values are always populated
                // so we could copy them without testing the flags
                // however in the future this could change, so lets
                // only copy when the flags tell us we can.
      
                if(bUpdateFlags & THROTTLE_FLAG)
                {
                  unThrottleIn = unThrottleInShared;
                }
    
                if(bUpdateFlags & AILERON_FLAG)
                {
                  unAileronIn = unAileronInShared;
                }
                
                if(bUpdateFlags & ELEVATOR_FLAG)
                {
                  unElevatorIn = unElevatorInShared;
                }
    
                if(bUpdateFlags & RUDDER_FLAG)
                {
                  unRudderIn = unRudderInShared;
                }
                
                 if(bUpdateFlags & GEAR_FLAG)
                {
                  unGearIn = unGearInShared;
                }
                
                if(bUpdateFlags & AUX_FLAG)
                {
                  unAuxIn = unAuxInShared;
                }
     
                // clear shared copy of updated flags as we have already taken the updates
                // we still have a local copy if we need to use it in bUpdateFlags
                bUpdateFlagsShared = 0;
    
                interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
                 // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
                 // service routines own these and could update them at any time. During the update, the 
                // shared copies may contain junk. Luckily we have our local copies to work with :-)
              }
  
              // do any processing from here onwards
              // only use the local values unGearIn, unThrottleIn and unAileronIn, the shared
              // variables unGearInShared, unThrottleInShared, unAileronInShared are always owned by 
              // the interrupt routines and should not be used in loop
  
              // the following code provides simple pass through 
              // this is a good initial test, the Arduino will pass through
              // receiver input as if the Arduino is not there.
              // This should be used to confirm the circuit and power
              // before attempting any custom processing in a project.
  
              // we are checking to see if the channel value has changed, this is indicated  
              // by the flags. For the simple pass through we don't really need this check,
              // but for a more complex project where a new signal requires significant processing
              // this allows us to only calculate new values when we have new inputs, rather than
              // on every cycle.

//THROTTLE
              if(bUpdateFlags & THROTTLE_FLAG)
              {
                if(servoThrottle.readMicroseconds() != unThrottleIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unThrottleIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_0, 0, pulselength);
                }
              }              

//AILERON
              if(bUpdateFlags & AILERON_FLAG)
              {
                if(servoAileron.readMicroseconds() != unAileronIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unAileronIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_1, 0, pulselength);
                }
              }
//ELEVATOR  
                if(bUpdateFlags & ELEVATOR_FLAG)
              {
                if(servoElevator.readMicroseconds() != unElevatorIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unElevatorIn, 800, 2000, 200, 510);
                  pwm.setPWM(servonum_2, 0, pulselength);
                }
              }
              
//RUDDER              if(bUpdateFlags & RUDDER_FLAG)
              {
                if(servoRudder.readMicroseconds() != unRudderIn)
                {
                  //servoRudder.writeMicroseconds(unRudderIn);
                  pulselength = map(unRudderIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_3, 0, pulselength);
                }
              }
//GEAR  
              if(bUpdateFlags & GEAR_FLAG)
              {
                if(servoGear.readMicroseconds() != unGearIn)
                {
                  //servoGear.writeMicroseconds(unGearIn);
                  pulselength = map(unGearIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_4, 0, pulselength);
                }
              }
//AUX  
              if(bUpdateFlags & AUX_FLAG)
              {
                if(servoAux.readMicroseconds() != unAuxIn)
                {
                  //servoAux.writeMicroseconds(unAuxIn);
                  pulselength = map(unAuxIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_5, 0, pulselength);
                }
              }
  
              bUpdateFlags = 0;
         
              GEAR = pulseIn(7, HIGH, 20000); 
            }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           

//AUTONOMOUS  /* Level Flight for 5 seconds */

     double current_time;
     double time_passed = 0;
     double prev_time = micros();
     
     while( (GEAR > 1500) && (time_passed<5000000) )
      {
          Serial.println("Level Flight 5 Seconds");
          Serial.println(time_passed);
/* Manual_Throttle */
         static uint16_t unThrottleIn;
          static uint8_t bUpdateFlags;
        // check shared update flags to see if any channels have a new signal
              if(bUpdateFlagsShared)
              {
                noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

                // take a local copy of which channels were updated in case we need to use this in the rest of loop
                bUpdateFlags = bUpdateFlagsShared;
    
                // in the current code, the shared values are always populated
                // so we could copy them without testing the flags
                // however in the future this could change, so lets
                // only copy when the flags tell us we can.
      
                if(bUpdateFlags & THROTTLE_FLAG)
                {
                  unThrottleIn = unThrottleInShared;
                }
                
                 bUpdateFlagsShared = 0;
    
                interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
                 // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
                 // service routines own these and could update them at any time. During the update, the 
                // shared copies may contain junk. Luckily we have our local copies to work with :-)
              }
              
              
//Manual_THROTTLE
              if(bUpdateFlags & THROTTLE_FLAG)
              {
                if(servoThrottle.readMicroseconds() != unThrottleIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unThrottleIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_0, 0, pulselength);
                }
              }
                
/* AUTONOMOUS */                
        double dT;
        
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        
        // Get the time of reading for rotation computations
        unsigned long t_now = millis();
        
        float FS_SEL = 131;
        
        float gyro_x = (gyro.gyro.x, - base_x_gyro)/FS_SEL;
        float gyro_y = (gyro.gyro.y, - base_y_gyro)/FS_SEL;
        float gyro_z = (gyro.gyro.z, - base_z_gyro)/FS_SEL;
        
        // Get raw acceleration values
        //float G_CONVERT = 16384;
        float accel_x = accel.acceleration.x;
        float accel_y = accel.acceleration.y;
        float accel_z = accel.acceleration.z;
        
        // Get angle values from accelerometer
        float RADIANS_TO_DEGREES = 180/3.14159;
        //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
      
        float accel_angle_z = 0;
        
        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);     
       
//AILERON
        
        pulselength = map(angle_y, -60, 60, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_1, 0, pulselength);
              //aileron_servo.write(val);
              prevPulselength = pulselength;
          }
          
//ELEVATOR
          
          pulselength = map(angle_x, -60, 60, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_2, 0, pulselength);
              //aileron_servo.write(val);
              prevPulselength = pulselength;
          }
              current_time = micros();
              time_passed = current_time - prev_time;
              GEAR = pulseIn(7, HIGH, 20000);
      } 
      
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* AUTONOMOUS */ /* Bank Left */

     float current_heading;
     float prev_heading = calc_heading();         // calc heading is at the bottom 
     float heading_change = 0;
     
     while( (GEAR > 1500) && (heading_change<90) )
      {
        
          Serial.println("banking left");
          Serial.println(heading_change);
          
          
/* Manual_Throttle */
         static uint16_t unThrottleIn;
          static uint8_t bUpdateFlags;
        // check shared update flags to see if any channels have a new signal
              if(bUpdateFlagsShared)
              {
                noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

                // take a local copy of which channels were updated in case we need to use this in the rest of loop
                bUpdateFlags = bUpdateFlagsShared;
    
                // in the current code, the shared values are always populated
                // so we could copy them without testing the flags
                // however in the future this could change, so lets
                // only copy when the flags tell us we can.
      
                if(bUpdateFlags & THROTTLE_FLAG)
                {
                  unThrottleIn = unThrottleInShared;
                }
                
                 bUpdateFlagsShared = 0;
    
                interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
                 // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
                 // service routines own these and could update them at any time. During the update, the 
                // shared copies may contain junk. Luckily we have our local copies to work with :-)
              }
              
              
//Manual_THROTTLE
              if(bUpdateFlags & THROTTLE_FLAG)
              {
                if(servoThrottle.readMicroseconds() != unThrottleIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unThrottleIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_0, 0, pulselength);
                }
              }
                
/* AUTONOMOUS */                
        double dT;
        
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        
        // Get the time of reading for rotation computations
        unsigned long t_now = millis();
        
        float FS_SEL = 131;
        
        float gyro_x = (gyro.gyro.x, - base_x_gyro)/FS_SEL;
        float gyro_y = (gyro.gyro.y, - base_y_gyro)/FS_SEL;
        float gyro_z = (gyro.gyro.z, - base_z_gyro)/FS_SEL;
        
        // Get raw acceleration values
        //float G_CONVERT = 16384;
        float accel_x = accel.acceleration.x;
        float accel_y = accel.acceleration.y;
        float accel_z = accel.acceleration.z;
        
        // Get angle values from accelerometer
        float RADIANS_TO_DEGREES = 180/3.14159;
        //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
      
        float accel_angle_z = 0;
        
        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);     
          
//AILERON

        angle_y = angle_y + 30;             // set airplane to 30 degree bank left
        pulselength = map(angle_y, -70, 70, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {                                                 // depending on the wind direction relative to take-off we may want to bank right for a right pattern instead. We can then just adjust the while loop to say while(angle)y < 30)
               pwm.setPWM(servonum_1, 0, pulselength);      // make the airplane think it is in a 30 degrees bank to the right so that it will bank the wing left by 30 instead.                                                         
               prevPulselength = pulselength;
          }
      
//ELEVATOR

          angle_x = angle_x - 10;            // set airplane to 10 degree climb 
          pulselength = map(angle_x, -70, 70, SERVOMAX, SERVOMIN);  
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_2, 0, pulselength);
              prevPulselength = pulselength;
          }      

              current_heading = calc_heading();      
              heading_change = current_heading - prev_heading;            
              GEAR = pulseIn(7, HIGH, 20000);
      } 
      
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////           

//AUTONOMOUS  /* Level Flight for 10 seconds */

     current_time;
     time_passed = 0;
     prev_time = micros();
     
     while( (GEAR > 1500) && (time_passed<10000000) )
      {
          Serial.println("Level Flight 10 Seconds");
          Serial.println(time_passed);
/* Manual_Throttle */
         static uint16_t unThrottleIn;
          static uint8_t bUpdateFlags;
        // check shared update flags to see if any channels have a new signal
              if(bUpdateFlagsShared)
              {
                noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

                // take a local copy of which channels were updated in case we need to use this in the rest of loop
                bUpdateFlags = bUpdateFlagsShared;
    
                // in the current code, the shared values are always populated
                // so we could copy them without testing the flags
                // however in the future this could change, so lets
                // only copy when the flags tell us we can.
      
                if(bUpdateFlags & THROTTLE_FLAG)
                {
                  unThrottleIn = unThrottleInShared;
                }
                
                 bUpdateFlagsShared = 0;
    
                interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
                 // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
                 // service routines own these and could update them at any time. During the update, the 
                // shared copies may contain junk. Luckily we have our local copies to work with :-)
              }
              
              
//Manual_THROTTLE
              if(bUpdateFlags & THROTTLE_FLAG)
              {
                if(servoThrottle.readMicroseconds() != unThrottleIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unThrottleIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_0, 0, pulselength);
                }
              }
                
/* AUTONOMOUS */                
        double dT;
        
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        
        // Get the time of reading for rotation computations
        unsigned long t_now = millis();
        
        float FS_SEL = 131;
        
        float gyro_x = (gyro.gyro.x, - base_x_gyro)/FS_SEL;
        float gyro_y = (gyro.gyro.y, - base_y_gyro)/FS_SEL;
        float gyro_z = (gyro.gyro.z, - base_z_gyro)/FS_SEL;
        
        // Get raw acceleration values
        //float G_CONVERT = 16384;
        float accel_x = accel.acceleration.x;
        float accel_y = accel.acceleration.y;
        float accel_z = accel.acceleration.z;
        
        // Get angle values from accelerometer
        float RADIANS_TO_DEGREES = 180/3.14159;
        //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
      
        float accel_angle_z = 0;
        
        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);     
       
//AILERON
        
        pulselength = map(angle_y, -60, 60, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_1, 0, pulselength);
              //aileron_servo.write(val);
              prevPulselength = pulselength;
          }
          
//ELEVATOR
          
          pulselength = map(angle_x, -60, 60, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_2, 0, pulselength);
              //aileron_servo.write(val);
              prevPulselength = pulselength;
          }
              current_time = micros();
              time_passed = current_time - prev_time;
              GEAR = pulseIn(7, HIGH, 20000);
      } 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* AUTONOMOUS */ /* Bank Left */

     current_heading;
     prev_heading = calc_heading();         // calc heading is at the bottom 
     heading_change = 0;
     
     while( (GEAR > 1500) && (heading_change<90) )
      {
        
          Serial.println("banking left");
          Serial.println(heading_change);
          
          
/* Manual_Throttle */
         static uint16_t unThrottleIn;
          static uint8_t bUpdateFlags;
        // check shared update flags to see if any channels have a new signal
              if(bUpdateFlagsShared)
              {
                noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

                // take a local copy of which channels were updated in case we need to use this in the rest of loop
                bUpdateFlags = bUpdateFlagsShared;
    
                // in the current code, the shared values are always populated
                // so we could copy them without testing the flags
                // however in the future this could change, so lets
                // only copy when the flags tell us we can.
      
                if(bUpdateFlags & THROTTLE_FLAG)
                {
                  unThrottleIn = unThrottleInShared;
                }
                
                 bUpdateFlagsShared = 0;
    
                interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
                 // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
                 // service routines own these and could update them at any time. During the update, the 
                // shared copies may contain junk. Luckily we have our local copies to work with :-)
              }
              
              
//Manual_THROTTLE
              if(bUpdateFlags & THROTTLE_FLAG)
              {
                if(servoThrottle.readMicroseconds() != unThrottleIn)
                {
                  //servoAileron.writeMicroseconds(unAileronIn);
                  pulselength = map(unThrottleIn, 800, 2000, SERVOMIN, SERVOMAX);
                  pwm.setPWM(servonum_0, 0, pulselength);
                }
              }
                
/* AUTONOMOUS */                
        double dT;
        
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        
        // Get the time of reading for rotation computations
        unsigned long t_now = millis();
        
        float FS_SEL = 131;
        
        float gyro_x = (gyro.gyro.x, - base_x_gyro)/FS_SEL;
        float gyro_y = (gyro.gyro.y, - base_y_gyro)/FS_SEL;
        float gyro_z = (gyro.gyro.z, - base_z_gyro)/FS_SEL;
        
        // Get raw acceleration values
        //float G_CONVERT = 16384;
        float accel_x = accel.acceleration.x;
        float accel_y = accel.acceleration.y;
        float accel_z = accel.acceleration.z;
        
        // Get angle values from accelerometer
        float RADIANS_TO_DEGREES = 180/3.14159;
        //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
      
        float accel_angle_z = 0;
        
        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
        
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);     
          
//AILERON

        angle_y = angle_y + 30;             // set airplane to 30 degree bank left
        pulselength = map(angle_y, -70, 70, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {                                                 // depending on the wind direction relative to take-off we may want to bank right for a right pattern instead. We can then just adjust the while loop to say while(angle)y < 30)
               pwm.setPWM(servonum_1, 0, pulselength);      // make the airplane think it is in a 30 degrees bank to the right so that it will bank the wing left by 30 instead.                                                         
               prevPulselength = pulselength;
          }
      
//ELEVATOR

          angle_x = angle_x - 10;            // set airplane to 10 degree climb 
          pulselength = map(angle_x, -70, 70, SERVOMAX, SERVOMIN);        
        
          if (pulselength != prevPulselength)
          {
              pwm.setPWM(servonum_2, 0, pulselength);
              prevPulselength = pulselength;
          }      

              current_heading = calc_heading();      
              heading_change = current_heading - prev_heading;            
              GEAR = pulseIn(7, HIGH, 20000);
      } 
}


            // simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcAileron()
{
  if(digitalRead(AILERON_IN_PIN) == HIGH)
  { 
    ulAileronStart = micros();
  }
  else
  {
    unAileronInShared = (uint16_t)(micros() - ulAileronStart);
    bUpdateFlagsShared |= AILERON_FLAG;
  }
}

void calcElevator()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(ELEVATOR_IN_PIN) == HIGH)
  { 
    ulElevatorStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unElevatorInShared = (uint16_t)(micros() - ulElevatorStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= ELEVATOR_FLAG;
  }
}

void calcRudder()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(RUDDER_IN_PIN) == HIGH)
  { 
    ulRudderStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unRudderInShared = (uint16_t)(micros() - ulRudderStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= RUDDER_FLAG;
  }
}

void calcGear(){
  if(digitalRead(GEAR_IN_PIN) == HIGH)
  { 
    ulGearStart = micros();
  }
  else
  {
    unGearInShared = (uint16_t)(micros() - ulGearStart);
    bUpdateFlagsShared |= GEAR_FLAG;
  }
}

void calcAux()
{
  //if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(AUX_IN_PIN) == HIGH)
  { 
    ulAuxStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unAuxInShared = (uint16_t)(micros() - ulAuxStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= AUX_FLAG;
  }  
}

int calc_heading(void)
{
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(mag.magnetic.y, mag.magnetic.x)+PI;
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //float declinationAngle = 0.22;
    float declinationAngle = 0.05;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

    return headingDegrees;
}
  
  
  
