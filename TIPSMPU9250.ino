// ATmega328 source code for TIPS. 
// Authors: Charlotte Treffers & Luc van Wietmarschen, TU Delft
// Date: 14 June 2016
// 

#include "Wire.h"
#include "MPU6050_6axis_MotionApps20.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;

// ================================================================
// ===               DECLERATION OF VARIABLES                   ===
// ================================================================

#define LED_PIN 13
bool blinkState = false;// state of the LED
int newDataFlag = 0;    // flag for data

// Button debounce variables
const int buttonPin = 3;     // the number of the pushbutton pin
int state = LOW;             // the current state of the output
int lastButtonState = HIGH;  // the previous reading from the input pin
long lastDebounceTime = 0;   // the last time the output was toggled
long debounceDelay = 50;     // the debounce time

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
long interval;          // intervalcounter

// Offset
VectorFloat magBias = VectorFloat(12.374, 51.823, -57.35); // Bias offset for magnetometers
VectorFloat magScaleX = VectorFloat(1.583, 0.056, -0.044); // Scaling offsets for the magnetometers
VectorFloat magScaleY = VectorFloat(-0.104, 1.376, -0.01);
VectorFloat magScaleZ = VectorFloat(-0.182, 0.089 , 1.494);
VectorFloat magBiasF;                                      // Adjusted bias offset for magnetometers               

// Orientation/motion variables
Quaternion orientationChip;                         // [w, x, y, z]  quaternion orientation of chip to reference frame
VectorInt16 gyroReadout;                            // [x, y, z]     gyro sensor measurements
VectorFloat gyroFilteredF;                          // [x, y, z]     gyro sensor measurements (Float)
VectorInt16 aaReadout;                              // [x, y, z]     accel sensor measurements
VectorFloat aaReadoutF;                             // [x, y, z]     accel sensor measurements (Float)
VectorInt16 aaFiltered;                             // [x, y, z]     accel sensor measurements noise filtered 
VectorFloat aaFilteredF;                            // [x, y, z]     accel sensor measurements noise filtered (Float)
VectorFloat aaNormalizedF;                          // [x, y, z]     accel sensor measurements noise normalized (Float)
VectorInt16 aaReal;                                 // [x, y, z]     gravity-free accel sensor measurements
VectorInt16 aaWorld;                                // [x, y, z]     reference-frame accel sensor measurements
VectorFloat aaWorldF;                               // [x, y, z]     reference-frame accel sensor measurements as floats
VectorFloat aaWorldFiltF;                           // [x, y, z]     reference-frame accel sensor measurements filtered as floats
VectorFloat gravity;                                // [x, y, z]     gravity vector
VectorInt16 magReadout;                             // [x, y, z]     magnetometer Readout
VectorFloat magReadoutF;                            // [x, y, z]     magnetometer Readout in floats
VectorFloat magFilteredF;                           // [x, y, z]     magnetometer filtered in floats
VectorFloat magNormalizedF;                         // [x, y, z]     magnetometer normalized in floats
float magTemp;                                      // Temporary store var for magnetometer calibration

// Position variables
Quaternion oldOrientationChip = Quaternion(1, 0, 0, 0); //[w,x,y,z]      of chip orientation
Quaternion quadRotate;                              //[w,x,y,z]      of chip rotation during interval
VectorFloat tip;                                    //[x,y,z]        of probe tip
VectorFloat sensor;                                 //[x,y,z]        of sensor
VectorFloat sensor2tip;                             //[x,y,z]        from sensor to tip
VectorFloat normal;                                 //normal         of probe on surface

// LSB values
float lsbValueAccel = 1 / 16438.0;                  // LSB value     accelerometers
float lsbValueGyro = 1 / 131.0;                     // LSB value     gyrometers
float lsbValueMag = 0.6;                            // LSB value     magnetometers


// Complementary filter variables
float filterconstant = 0.9;                         // Filtercontant used for complementatry filter
VectorFloat aaAngle;                                // [x, y, z]     angle calculated by value of accel sensors
VectorFloat gyroAngle;                              // [x, y, z]     angle calculated by value of gyro sensors
VectorFloat angle = VectorFloat(0, 0, 0);           // [x, y, z]     angle calculated by combining accel and gyro sensors
VectorFloat oldAngle = VectorFloat(0, 0, 0);        // [x, y, z]     previous angle calculated by combining accel and gyro sensors

// Noise filter variables
VectorInt16 noiseParaAccel(200, 200, 250);          //parameter accelerometer
VectorInt16 noiseParaAccelSen(150, 150, 200);       //parameter accelerometer
float noiseParaAccelLow = 0.025 ;                   //parameter accelerometer low pass filter
VectorInt16 noiseParaGyro(35, 35, 35);              //parameter gyroscope
VectorInt16 noiseParaGyroSen(30, 30, 30);           //parameter gyroscope
float noiseParaGyroLow = 0.0;                       //parameter gyroscope low pass filter
VectorInt16 noiseParaMag(6, 6, 6);                  //parameter magnetometer
VectorInt16 noiseParaMagSen(6, 6, 6);               //parameter magnetometer
float noiseParaMagLow = 0.025;                      //parameter magnetometer low pass filter
VectorFloat nullVector = VectorFloat(0, 0, 0);      //null vector for reference

// Low pass filter variables
VectorFloat new_afilt = VectorFloat(0,0,0);
VectorFloat old_afilt;
VectorFloat new_aLP = VectorFloat(0, 0, 0);
VectorFloat old_aLP;
VectorFloat new_vfilt = VectorFloat(0,0,0);
VectorFloat old_vfilt;
VectorFloat new_vLP = VectorFloat(0, 0, 0);
VectorFloat old_vLP;
VectorFloat new_sfilt = VectorFloat(0,0,0);
VectorFloat old_sfilt;
VectorFloat new_sLP = VectorFloat(0, 0, 0);
VectorFloat old_sLP;

// Translation variables
float dT;                                           //secondes       duration of calculation interval
VectorFloat old_a;
VectorFloat new_a;
VectorFloat old_v;
VectorFloat new_v;
VectorFloat old_s;
VectorFloat new_s;

// Set up constants
VectorFloat sensorstart = VectorFloat(1, 1, 1);     //starting position of sensor relative to tip
VectorFloat normalstart = VectorFloat(0, 0, 1);     //startnormal of probe
float rad2deg = 180 / 3.141356;                     // radian to degree conversion constant
float deg2rad = 3.141356 / 180;                     // degree to radian conversion constant

// Calculation variables
unsigned long timer;                                // timer to measure loop durations
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;                 // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
// ================================================================
// ===                  Declaration functions                   ===
// ================================================================
void resetVars();
int buttonDebounce();
VectorFloat noiseFilter(VectorFloat *compareValue, VectorInt16 *value, VectorFloat *previousCorrect, float noiseParameterLowPass, VectorInt16 noiseParameterSen, VectorInt16 *noiseParameter  );
VectorFloat complementaryFilter(VectorFloat *gyroF, VectorFloat *aaF, VectorFloat *magnet, float dT);
VectorFloat getAaWorldFiltF(Quaternion orientationChip, VectorInt16 *aaFilteredF, float dT);
VectorFloat accelAngle(VectorFloat *aaF, VectorFloat *magnet);
float trapezoid(float new_data, float old_data, float old_out, float dT);
int float2int(float flo);
VectorFloat lowpassfilter(VectorFloat *prevX, VectorFloat *newU, VectorFloat *prevU, float alpha);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);// select pin for button
  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);
  mpu.initialize(); // initialize device
  devStatus = mpu.dmpInitialize(); // verify connection

  if (devStatus == 0) { // make sure it worked (returns 0 if so)
    mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
    mpuIntStatus = mpu.getIntStatus(); // enable Arduino interrupt detection
  
    Serial.println(F("DMP ready! Push button to start"));  // set DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  // get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);  // configure LED for output
  timer = micros();                                       //start timer
  interval = millis();                                    //start intervalcounter
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return;             // if programming failed, don't try to do anything

  if (millis() - interval > 1000) {  //MPU6050 falls a sleep after a while, avoid this by waking him up every second
    mpu.setDMPEnabled(true);
    interval = millis();             //"reset" intervalcounter
  }

  // debouncing button
  state = buttonDebounce();          //Check if button is pressed
  if (newDataFlag) {                 //after new data is put in vars, calculate
    if (!state) {                    //if device is not reading the starting position will be assumed
      aaReadoutF = aaReadout.getVectorFloat(); //convert acceleration to float
      aaFilteredF = noiseFilter(&aaFilteredF, &aaReadout, &aaFilteredF, noiseParaAccelLow, &noiseParaAccelSen, &noiseParaAccel); // acquire angles determined by the accelerometers and magnetometers
      resetVars();                   // put variables in starting position
    } else {                         //else start rotating and then translating the system
      //else start calculations
      
      //Offset magnetometer
      magBiasF.x = magReadout.x - magBias.x;
      magBiasF.y = magReadout.y - magBias.y;
      magBiasF.z = magReadout.z - magBias.z;
      magReadoutF.x = (magBiasF.x * magScaleX.x + magBiasF.y * magScaleX.y + magBiasF.z * magScaleX.z);
      magReadoutF.y = (magBiasF.x * magScaleY.x + magBiasF.y * magScaleY.y + magBiasF.z * magScaleY.z);
      magReadoutF.z = (magBiasF.x * magScaleZ.x + magBiasF.y * magScaleZ.y + magBiasF.z * magScaleZ.z);
      //Set magneto axis to our axis
      magTemp = magReadoutF.x;
      magReadoutF.x = magReadoutF.y;
      magReadoutF.y = magTemp;
      magReadoutF.z = -magReadoutF.z;

      //Offset accelerometer
      aaReadout.x = aaReadout.x + 65;
      aaReadout.y = aaReadout.y - 165;
      aaReadout.z = aaReadout.z + 13;

      //Offset gyroscopes
      gyroReadout.x = gyroReadout.x + 129;
      gyroReadout.y = gyroReadout.y - 57;
      gyroReadout.z = gyroReadout.z + 33;

      //noise filtering inputs
      magReadout.x = float2int(magReadoutF.x);
      magReadout.y = float2int(magReadoutF.y);
      magReadout.z = float2int(magReadoutF.z);
      aaFilteredF   = noiseFilter(&aaFilteredF,   &aaReadout,   &aaFilteredF,  noiseParaAccelLow, &noiseParaAccelSen, &noiseParaAccel);
      gyroFilteredF = noiseFilter(&gyroFilteredF, &gyroReadout, &nullVector,   noiseParaGyroLow,  &noiseParaGyroSen,  &noiseParaGyro);
      magFilteredF  = noiseFilter(&magFilteredF,  &magReadout,  &magFilteredF, noiseParaMagLow,   &noiseParaMagSen,   &noiseParaMag);

      //applying LSB values to gyros
      gyroFilteredF.x = gyroFilteredF.x * lsbValueGyro * deg2rad;
      gyroFilteredF.y = gyroFilteredF.y * lsbValueGyro * deg2rad;
      gyroFilteredF.z = gyroFilteredF.z * lsbValueGyro * deg2rad;
      //normalise magnetometers and accelerometers
      magNormalizedF = magFilteredF.getNormalized(); //only ratio is used, so LSB doenst matter
      aaNormalizedF = aaFilteredF.getNormalized();

      // calculate angle
      angle = complementaryFilter(&gyroFilteredF, &aaNormalizedF, &magNormalizedF, dT, oldAngle);    //calculate new angle of sensor
      oldAngle = angle;             //make oldangle the previous calculated angle
      //yaw doesn't work properly, this sets them for reference to use in other functions
//      angle.x = 0;
//      angle.y = 0;
//      angle.z = 180;              //yaw doesn't work properly

      // Calculate Quaternion
      orientationChip = angle.getQuat();

      //rotating
      quadRotate = orientationChip.getProduct(oldOrientationChip.getConjugate()); // finding the rotation shift during the interval, disp * q1 = q2 --> disp = q2*inv(q1)
      sensor2tip.rotate(&quadRotate);         //rotate the sensor to tip translation vector with the new rotation
      normal.rotate(&quadRotate);             //rotate the normal with the new rotation
      oldOrientationChip = orientationChip;   //store the orientation

      //translating
      aaWorldFiltF = getAaWorldFiltF(&orientationChip, &aaFilteredF, dT);
//      aaFilteredF.z = aaFilteredF.z - 16384; //used when angle is not active and chip is positioned with z facing up
      
      // updating acceleration
      old_a = new_a;                    // storing old data
      new_a = aaWorldFiltF;             // reading out new data
//      new_a = aaFilterdF;                    //used when angle is not active and chip is positioned with z facing up
      // filtering aceleration
      old_aLP = new_aLP;                // storing old drift offset
      old_afilt = new_afilt;            // storing old filtered data
      new_aLP = lowpassfilter(&old_aLP, &new_a , &old_a, 0.05); // calculate new drift offset
      new_afilt.x = new_a.x - new_aLP.x;// subtract drift from data
      new_afilt.y = new_a.y - new_aLP.y;
      new_afilt.z = new_a.z - new_aLP.z;

      // updating velocity
      old_v = new_v;                    // storing old data
      new_v.x = trapezoid(new_afilt.x, old_afilt.x, old_v.x, dT); // reading out new data
      new_v.y = trapezoid(new_afilt.y, old_afilt.y, old_v.y, dT);
      new_v.z = trapezoid(new_afilt.z, old_afilt.z, old_v.z, dT);
      // filtering v
      old_vLP = new_vLP;                // storing old drift offset
      old_vfilt = new_vfilt;            // storing old filtered data
      new_vLP = lowpassfilter(&old_vLP, &new_v , &old_v, 0.05); // calculate new drift offset
      new_vfilt.x = new_v.x - new_vLP.x;// subtract drift from data
      new_vfilt.y = new_v.y - new_vLP.y;
      new_vfilt.z = new_v.z - new_vLP.z;
      
      // updating displacement  
      old_s = new_s;                    // storing old data
      new_s.x = trapezoid(new_vfilt.x, old_vfilt.x, old_s.x, dT); // reading out new data
      new_s.y = trapezoid(new_vfilt.y, old_vfilt.y, old_s.y, dT);
      new_s.z = trapezoid(new_vfilt.z, old_vfilt.z, old_s.z, dT);
      //filtering s
      old_sLP = new_sLP;                // storing old drift offset
      old_sfilt = new_sfilt;            // storing old filtered data
      new_sLP = lowpassfilter(&old_sLP, &new_s , &old_s, 0.05); // calculate new drift offset
      new_sfilt.x = new_s.x - new_sLP.x;// subtract drift from data
      new_sfilt.y = new_s.y - new_sLP.y;
      new_sfilt.z = new_s.z - new_sLP.z;

      sensor = sensor.getAddition(new_s);      //translate the sensor in the reference frame
      tip = sensor.getAddition(sensor2tip);    //translate from sensor to tip position

      // printing to computer
      Serial.print(F("xpos:"));
      Serial.println(tip.x, DEC);
      Serial.print(F("ypos:"));
      Serial.println(tip.y, DEC);
      Serial.print(F("zpos:"));
      Serial.println(tip.z, DEC);
      Serial.print(F("xnor:"));
      Serial.println(normal.x, DEC);
      Serial.print(F("ynor:"));
      Serial.println(normal.y, DEC);
      Serial.print(F("znor:"));
      Serial.println(normal.z, DEC);

      newDataFlag = 0;                           //reset newDataFlag
    }
  }

  mpuInterrupt = false;                          // reset interrupt flag
  interval = millis();                           // reset intervalcounter
  mpuIntStatus = mpu.getIntStatus();             // get INT_STATUS byte
  fifoCount = mpu.getFIFOCount();                // get current FIFO count
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {// check for overflow
    mpu.resetFIFO();                             // reset so we can continue cleanly
    //    Serial.println(F("FIFO overflow!"));

  } else if (mpuIntStatus & 0x02) {              // else check for DMP data ready interrupt
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();            // wait for correct available data length
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);    // read a packet from FIFO
    // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // actually read data from MPU9250
    mpu.getMotion9(&aaReadout.x, &aaReadout.y, &aaReadout.z, &gyroReadout.x, &gyroReadout.y, &gyroReadout.z, &magReadout.x, &magReadout.y, &magReadout.z);

    blinkState = !blinkState;                    // blink LED to indicate looping
    digitalWrite(LED_PIN, blinkState);
    newDataFlag = 1;                             //set flag to indicate new data
    dT = (micros() - timer) / 1000000.0;         // update dT, division to convert from micro to seconds
    timer = micros();                            // update timer
  }
}


// ================================================================
// ===                      FUNCTIONS                           ===
// ================================================================
int buttonDebounce() {
  int reading = digitalRead(buttonPin);   // read the state of the switch
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember the time
  if (reading == HIGH && lastButtonState == LOW && (micros() - lastDebounceTime > debounceDelay)) {
    if (state == HIGH) {
      state = LOW;
	  Serial.println(F("stopStream"));           // used to let Matlab known that the measuring is stopped
    }
    else{
      state = HIGH;
	  Serial.println(F("startStream"));           // used to let Matlab known that the measuring is started
	}
    lastDebounceTime = micros();
  }
  lastButtonState = reading;              // save the reading
  return state;
}

void resetVars() {
  sensor = sensorstart;                        // sensor starting position
  normal = normalstart;                        // normal starting position
  oldOrientationChip = Quaternion(1, 0, 0, 0); // reset orientation
  newDataFlag = 0;                             // reset count
  //find starting angle
  aaReadoutF = aaReadout.getVectorFloat();     // make acceleration float
  aaNormalizedF = aaReadoutF.getNormalized();  // normalize acceleration
  magReadoutF = magReadout.getVectorFloat();   // make magnetometer float
  magNormalizedF = magReadoutF.getNormalized();// normalise magnetometer
  oldAngle = accelAngle(&aaNormalizedF, &magNormalizedF); //store old angle
  aaFiltered = aaReadout;                      // set starting aaAngle to readout value
  //reset filter values
  new_a = VectorFloat(0.0, 0.0, 0.0);
  new_v = VectorFloat(0.0, 0.0, 0.0);
  new_s = VectorFloat(0.0, 0.0, 0.0);
  new_aLP = VectorFloat(0, 0, 0);
  new_vLP = VectorFloat(0, 0, 0);
  new_sLP = VectorFloat(0, 0, 0);
  new_afilt = aaFiltered.getVectorFloat();
  new_vfilt = VectorFloat(0, 0, 0);
  new_sfilt = VectorFloat(0, 0, 0);
}

VectorFloat noiseFilter(VectorFloat *compareValue, VectorInt16 *value, VectorFloat *previousCorrect, float noiseParameterLowPass, VectorInt16 *noiseParameterSen, VectorInt16 *noiseParameter  ) {
  VectorFloat filteredValue;
  if (value->x < compareValue->x - noiseParameter->x | value->x > compareValue->x + noiseParameter->x) {
    filteredValue.x = value->x;
  } else {
    if (value->x < compareValue->x - noiseParameterSen->x | value->x > compareValue->x + noiseParameterSen->x) {
      filteredValue.x = (previousCorrect->x + value->x) / 2;
    } else {
      filteredValue.x = ((float)((1.0 - noiseParameterLowPass) * previousCorrect->x + noiseParameterLowPass * value->x));
    }
  }

  if (value->y < compareValue->y - noiseParameter->y | value->y > compareValue->y + noiseParameter->y) {
    filteredValue.y = value->y;
  } else {
    if (value->y < compareValue->y - noiseParameterSen->y | value->y > compareValue->y + noiseParameterSen->y) {
      filteredValue.y = (previousCorrect->y + value->y) / 2;
    } else {
      filteredValue.y = ((float)((1.0 - noiseParameterLowPass) * previousCorrect->y + noiseParameterLowPass * value->y));
    }
  }

  if (value->z < compareValue->z - noiseParameter->z | value->z > compareValue->z + noiseParameter->z) {
    filteredValue.z = value->z;
  } else {
    if (value->z < compareValue->z - noiseParameterSen->z | value->z > compareValue->z + noiseParameterSen->z) {
      filteredValue.z = (previousCorrect->z + value->z) / 2;
    } else {
      filteredValue.z = ((float)((1.0 - noiseParameterLowPass) * previousCorrect->z + noiseParameterLowPass * value->z));
    }

  }
  return filteredValue;
}


VectorFloat complementaryFilter(VectorFloat *gyroF, VectorFloat *aaF, VectorFloat *magnet, float dT, VectorFloat oldAngle) {
  //Calculate angles with accelerometers and magnetometer
  aaAngle.x = atan2(aaF->y, sqrt(pow(aaF->x, 2) + pow(aaF->z, 2))) * rad2deg; //rol
  aaAngle.y = atan(aaF->x / sqrt(pow(aaF->y, 2) + pow(aaF->z, 2))) * rad2deg; //pitch

  //Calculate angles with gyroscopes
  gyroAngle.x = gyroF->x * dT + oldAngle.x;
  gyroAngle.y = gyroF->y * dT + oldAngle.y;

  //Calculate new angles
  angle.x = filterconstant * (gyroAngle.x) + (1.0 - filterconstant) * (aaAngle.x);
  angle.y = filterconstant * (gyroAngle.y) + (1.0 - filterconstant) * (aaAngle.y);

  //Calculate yaw
  angle.z = atan2((magnet->x * sin(angle.x * deg2rad) * sin(angle.y * deg2rad) + magnet->y * cos(angle.x * deg2rad) - magnet->z * sin(angle.x * deg2rad) * cos(angle.y * deg2rad)), (magnet->x * cos(angle.y * deg2rad) + magnet->z * sin(angle.y * deg2rad)));
  angle.z = angle.z * rad2deg;
  return angle;
}

VectorFloat accelAngle(VectorFloat *aaF, VectorFloat *magnet) {
  aaAngle.x = atan2(aaF->y, sqrt(pow(aaF->x, 2) + pow(aaF->z, 2))); //rol
  aaAngle.y = atan(aaF->x / sqrt(pow(aaF->y, 2) + pow(aaF->z, 2))); //pitch
  aaAngle.z = atan2((magnet->x * cos(aaAngle.y) + magnet->y * sin(aaAngle.x) * sin(aaAngle.y) + magnet->z * cos(aaAngle.x) * sin(aaAngle.y)), (-magnet->y * cos(aaAngle.x) + magnet->z * sin(aaAngle.x))) ;
  aaAngle.x = aaAngle.x * rad2deg;
  aaAngle.y = aaAngle.y * rad2deg;
  aaAngle.z = aaAngle.z * rad2deg;
  return aaAngle;
}

VectorFloat getAaWorldFiltF(Quaternion *orientationChip, VectorFloat *aaFilteredF, float dT) {
  mpu.dmpGetGravity(&gravity, orientationChip);         //get gravity in gravity(x,y,z)
  aaFiltered.x = float2int(aaFilteredF->x);             // make floats
  aaFiltered.y = float2int(aaFilteredF->y);
  aaFiltered.z = float2int(aaFilteredF->z);
  mpu.dmpGetLinearAccel(&aaReal, &aaFiltered, &gravity);//get gravity free acceleration in aaReal(x,y,z)
  aaWorld = aaReal.getRotated(orientationChip);         //get gravity free acceleration in world reference frame in aaWorld(x,y,z)
  aaWorldFiltF.x = aaWorld.x * lsbValueAccel;            //convert value of accel sensor to m/s
  aaWorldFiltF.y = aaWorld.y * lsbValueAccel;
  aaWorldFiltF.z = aaWorld.z * lsbValueAccel;
  return aaWorldFiltF;
}

int float2int(float flo) {
  int in;
  if (flo < 0) { // check for negative value
    in = (int)(flo - 0.5); // round to nearest negative value
  }
  else {
    in = (int)(flo + 0.5); // round to nearest positive value
  }
  return in;
}

float trapezoid(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data);
  return out;
}

VectorFloat lowpassfilter(VectorFloat *prevX, VectorFloat *newU, VectorFloat *prevU, float alpha) {
  VectorFloat newX;
  newX.x = ((1 - alpha) * prevX->x + alpha * prevU->x);
  newX.y = ((1 - alpha) * prevX->y + alpha * prevU->y);
  newX.z = ((1 - alpha) * prevX->z + alpha * prevU->z);
  return newX;
}

