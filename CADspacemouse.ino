#include "src/SparkFunMPU9250-DMP.h"

MPU9250_DMP imu;

int scale = 10;

int rOffset = 175;
int pOffset = 4;
int yOffset = 143;

int rTresh = 15;
int pTresh = 15;
int yTresh = 15;

int fixedRoll;
int fixedPitch;
int fixedYaw;

int prevRoll, prevPitch, prevYaw;
bool stateChanged = false;
bool offsetsInited = false;
unsigned long lastTime = millis();

char zeros[4];
char rollBytes[4];
char pitchBytes[4];
char yawBytes[4];

void setup() 
{
  Serial.begin(9600);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              42); // Set DMP FIFO rate to 42 Hz

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); // Enable all sensors

  // Use setGyroFSR() and setAccelFSR() to configure the gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(1000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  imu.setLPF(42); // Set low-pass filter corner frequency(188, 98, 42, 20, 10, 5 Hz)
  imu.setSampleRate(420); // Set sample accel/gyro rate to 10Hz (from 4Hz to 1kHz)
  imu.setCompassSampleRate(42); // Set mag sample rate to 10Hz (1-100 Hz)

  zeros[0] = 0x08; // high byte, first 0x0 is unused
  zeros[1] = 0x00;
  zeros[2] = 0x00;
  zeros[3] = 0x00; // low byte
}

void calcOffsets(){
  rOffset = prevRoll;
  pOffset = prevPitch;
  yOffset = prevYaw;
  offsetsInited = true;
}

bool checkAxisNonLinear(int offset){
  return (offset < 90) || (offset > 270);
}

int getTresholded(int value, int threshold) {
  if(abs(value)*scale > threshold)
    return value*scale;
  else
    return 0;
}

void loop() 
{
  if (Serial.available() > 0) {
    char firstChar = Serial.read();
    if(firstChar == 'v')
      Serial.println("vMAGELLAN\r");
  }

   // Check for new data
   // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    if ( imu.dataReady() )
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      calcTranslations();
    }
  }
}

void calcTranslations() {
  if(stateChanged) {
    lastTime = millis();
    stateChanged = false;
  }
  else
    if((millis() - lastTime) > 1000) // 1 seconds
      calcOffsets();

  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  fixedRoll = imu.roll;
  fixedPitch = imu.pitch;
  fixedYaw = imu.yaw;
 
  if((fixedRoll > 180) && checkAxisNonLinear(rOffset))
    fixedRoll -= 358;
  if((fixedPitch > 180) && checkAxisNonLinear(pOffset))
    fixedPitch -= 358;
  if((fixedYaw > 180) && checkAxisNonLinear(yOffset))
    fixedYaw -= 358;
  
  if(offsetsInited)
      printIMUData();

  if(fixedRoll != prevRoll) {          
    prevRoll = fixedRoll;
    stateChanged = true;
  }
  if(fixedPitch != prevPitch) {          
    prevPitch = fixedPitch;
    stateChanged = true;
  }
  if(fixedYaw != prevYaw) {        
    prevYaw = fixedYaw;
    stateChanged = true;
  }

}

void intToBytes(int value, char *bytes) {
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;

  bytes[0] -= 0x08;
}


void printIMUData(void) {
  intToBytes(getTresholded(fixedRoll - rOffset, rTresh), rollBytes);
  intToBytes(getTresholded(fixedPitch - pOffset, pTresh), pitchBytes);
  intToBytes(getTresholded(fixedYaw - yOffset, yTresh), yawBytes);
  
  Serial.print('d');
  Serial.write(zeros, 4); //Translate X
  Serial.write(zeros, 4); //Translate Y
  Serial.write(zeros, 4); //Translate Z

  Serial.write(pitchBytes, 4); //Rotate Y
  Serial.write(rollBytes, 4); //Rotate X
  Serial.write(yawBytes, 4); //Rotate Z

  Serial.print('\r');

// int mx = imu.calcMag(imu.mx)/10;
// int my = imu.calcMag(imu.my)/10;
// int mz = imu.calcMag(imu.mz)/10;
// Serial.println(String(getTresholded(fixedRoll - rOffset, rTresh)) + ", " 
//                   + String(getTresholded(fixedPitch - pOffset, pTresh)) + ", " 
//                  + String(getTresholded(fixedYaw - yOffset, yTresh)));
}