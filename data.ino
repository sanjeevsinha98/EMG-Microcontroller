#include "CurieIMU.h"

int ax, ay, az, gx, gy, gz, ADC, input;
float axf, ayf, azf, gxf, gyf, gzf, mag_a;
float ADC_volt, bp_output, EMG_rect;
float timei, timef, time_elapsed;

float thirdOrderIIR_highpass(float sample)
{
  static const float a[4] = {1.0,-2.37409474, 1.92935567, -0.53207537}; // ADD A VALUES HERE
  static const float b[4] = {0.72944072, -2.18832217, 2.18832217, -0.72944072};// ADD B VALUES HERE

  // x array for holding recent inputs (newest input as index 0, delay of 1 at index 1, etc.
  static float x[4] = {0};
  // x array for holding recent inputs (newest input as index 0, delay of 1 at index 1, etc.
  static float y[4] = {0};

  x[0] = sample;

  // Calculate the output filtered signal based on a weighted sum of previous inputs/outputs
  y[0] = (b[0]*x[0]+b[1]*x[1]+b[2]*x[2]+b[3]*x[3])-(a[1]*y[1]+a[2]*y[2]+a[3]*y[3]);
  y[0] /= a[0];

  // Shift the input signals by one timestep to prepare for the next call to this function
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];

  // Shift the previously calculated output signals by one time step to prepare for the next call to this function
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];

  return y[0];
}

float thirdOrderIIR_lowpass(float sample)
{
  static const float a[4] = {1.0, pow(-2.77555756, -16), pow(3.333, -1), pow(-1.85037171, -17)};// ADD A VALUES HERE
  static const float b[4] = {0.16666667, 0.5, 0.5, 0.16666667};// ADD B VALUES HERE

  // x array for holding recent inputs (newest input as index 0, delay of 1 at index 1, etc.
  static float x[4] = {0};
  // x array for holding recent inputs (newest input as index 0, delay of 1 at index 1, etc.
  static float y[4] = {0};

  x[0] = sample;

  // Calculate the output filtered signal based on a weighted sum of previous inputs/outputs
  y[0] = (b[0]*x[0]+b[1]*x[1]+b[2]*x[2]+b[3]*x[3])-(a[1]*y[1]+a[2]*y[2]+a[3]*y[3]);
  y[0] /= a[0];

  // Shift the input signals by one timestep to prepare for the next call to this function
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];

  // Shift the previously calculated output signals by one time step to prepare for the next call to this function
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];

  return y[0];
}

void setup() {
  Serial.begin(115200);// initialize baud rate 
  CurieIMU.begin(); // initialize inertial measurement unit
  CurieIMU.setGyroRange(2000); //set degree range 2000 degrees
  CurieIMU.setAccelerometerRange(4); //set acc. range to 4g

  Serial.begin(115200);
  while(!Serial){}
  Serial.print("Input a to read data:\n");
}

void loop() {
  while(Serial.available() > 0) 
    input = Serial.read();
  
  CurieIMU.readAccelerometer(ax, ay, az);
  CurieIMU.readGyro(gx, gy, gz);

  /*
  //Accelerometer
  //Convert Raw Value to g's
  axf = (ax/32768.0) * CurieIMU.getAccelerometerRange();
  ayf = (ay/32768.0) * CurieIMU.getAccelerometerRange();
  azf = (az/32768.0) * CurieIMU.getAccelerometerRange();
  Serial.print(axf); Serial.print("\t");
  Serial.print(ayf); Serial.print("\t");
  Serial.print(azf); Serial.print("\t");
  Serial.print("\t");
  //Magnitude of Accelerometer, maybe useful for determining position of forearm
  /*mag_a = sqrt(pow(axf, 2) + pow(ayf, 2) + pow(azf, 2));
  Serial.print(mag_a);
  Serial.print("\t");
  */

  //Gyroscope
  //Convert Raw Values to Degrees/second
  gxf = (gx/32768.9) * CurieIMU.getGyroRange(); 
  gyf = (gy/32768.9) * CurieIMU.getGyroRange();
  gzf = (gz/32768.9) * CurieIMU.getGyroRange();
  //Serial.print(gxf); Serial.print("\t");
  //Serial.print(gyf); Serial.print("\t");
  //Serial.print(gzf); Serial.print("\t");

  
  //EMG Shield
  ADC = analogRead(A0);
  ADC_volt = (ADC) * (3.3/1024) - 1.5; //Converts Raw Value to Volts
  bp_output = thirdOrderIIR_lowpass(thirdOrderIIR_highpass(ADC_volt)); //Apply High and low pass filters
  EMG_rect = abs(bp_output); //Rectify the EMG signal
  //Serial.print("\t");
  //Serial.print(ADC);
  //Serial.print("\t");
  

  //Controls Using Only Gyroscope
  //(if the Arduino lays flat on the forearm with z axis pointing up and down)
  //Rotate around x axis to emulate up and down buttons
  if(gxf >= 800)
  {
    Serial.print("U");
    delay(140); //Any button that requires a strong force should have a longer delay.
  }
  else if(gxf <= -800)
  {
    Serial.print("D");
    delay(140);
  }
  //Apply downward force (rotate downward around y-axis) to emulate b button
  else if(gyf >= 200)
  {
    Serial.print("B");
    delay(140);
  }
  //Move left or right (rotate around z-axis) to emulate left and right buttons  
  else if(gzf >= 100)
  {
    if(EMG_rect >= 0.1)
    {
      Serial.print("L");
      delay(50);
    }
    else
    {
      Serial.print("L");
      delay(5);  
    }
  }
    
  else if(gzf <= -100)
  {
    if(EMG_rect >= 0.1)
    {
      Serial.print("R");
      delay(50);
    }
    else
    {
      Serial.print("R");
      delay(5);  
    }
  }
  else if(EMG_rect >= 0.3)
  {
    Serial.print("A");
    delay(5);
  }
  else
  {
    Serial.print("N"); //No input
  }

  timef = micros();
  time_elapsed = timef - timei;
  timei = timef;

  //Useful info for debugging purposes
  Serial.print("\n");
  Serial.print(time_elapsed);
  Serial.print("\t");
  Serial.print(gxf);
  Serial.print("\t");
  Serial.print(gyf);
  Serial.print("\t");
  Serial.print(gzf);
  Serial.print("\t");
  Serial.print(EMG_rect);

  Serial.print("\n");
  delay(10);

}
