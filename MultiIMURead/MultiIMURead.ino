///////////////////////////////////////////////////////////////////////////////////////
/*Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

 
///////////////////////////////////////////////////////////////////////////////////////
//Support
///////////////////////////////////////////////////////////////////////////////////////
Website: http://www.brokking.net/imu.html
Youtube: https://youtu.be/4BoIE8YQwM8
Version: 1.0 (May 5, 2016)

///////////////////////////////////////////////////////////////////////////////////////
//Connections
///////////////////////////////////////////////////////////////////////////////////////
Power (5V) is provided to the Arduino pro mini by the FTDI programmer

Gyro - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5

LCD  - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5
*//////////////////////////////////////////////////////////////////////////////////////

//Include LCD and I2C library
#include <Wire.h>

//Declaring some global variables
int gyro_x[6], gyro_y[6], gyro_z[6];
long acc_x[6], acc_y[6], acc_z[6], acc_total_vector[6];

int temperature[6];
long gyro_x_cal[6], gyro_y_cal[6], gyro_z_cal[6];

long loop_timer;
float angle_pitch[6], angle_roll[6];
int angle_pitch_buffer[6], angle_roll_buffer[6];
boolean set_gyro_angles[6];
float angle_roll_acc[6], angle_pitch_acc[6];
float angle_pitch_output[6], angle_roll_output[6];

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.


void setup() {
  //Setting up the IMUs
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                                 //Use only for debugging

  for (int i = 2; i <=7; i++){
  pinMode(i, OUTPUT);
  digitalWrite(i,HIGH);
  }

  for (int i = 2; i <=7; i++){
  digitalWrite(i,LOW); //turn on the IMU 
  setup_mpu_6050_registers();     //Setup the registers of the MPU-6050  (i-2 because IMU_num are 0-5 and pin are 2-7)
  digitalWrite(i,HIGH); // turn off the IMU
  }


  int calIter = 200; //Calibration Interations (different to cal_int)
  
  for(int IMU_num = 0; IMU_num <=5; IMU_num++){
  //Calibrating the IMUs
  for (int cal_int = 0; cal_int < calIter ; cal_int ++){                  //Run this code 2000 times
    Serial.print("Calibrating IMU ");Serial.println(IMU_num);
//    digitalWrite(i+2,LOW); // setting the right IMU to be read (i_2 because IMU_num are 0-5 and pin are 2-7))
    read_mpu_6050_data(IMU_num);                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal[IMU_num] += gyro_x[IMU_num];                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal[IMU_num] += gyro_y[IMU_num];                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal[IMU_num] += gyro_z[IMU_num];                                              //Add the gyro z-axis offset to the gyro_z_cal variable
//    digitalWrite(i+2,HIGH);
    delay(1);
    //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal[IMU_num] /= calIter;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal[IMU_num] /= calIter;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal[IMU_num] /= calIter;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  Serial.print("Calibrating IMU ");Serial.print(IMU_num);Serial.print(" complete ");

  loop_timer = micros();                                               //Reset the loop timer
  }
}

void loop(){
  for(int IMU_num = 0; IMU_num <=5; IMU_num++)
  {
//    read_mpu_6050_data(i);
   computeAngle(IMU_num);

  }
  
//  //RUN FUNCTION TO DO sensor read 
//  float pitch1 = angle_pitch_output;
//  float roll1 = angle_roll_output;

//angle_pitch_output
//angle_roll_output

  //Displaying the data
  Serial.print("\n\nIMU1\t");
//  Serial.print("Gyro x: "); Serial.print(gyro_x[0]);
//  Serial.print("  \tGyro y: "); Serial.print(gyro_y[0]);
//  Serial.print("  \tGyro z: "); Serial.print(gyro_z[0]);
  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[0]);
  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[0]);
  
//  Serial.print("\nIMU2\t");
////  Serial.print("Gyro x: "); Serial.print(gyro_x[1]);
////  Serial.print("  \tGyro y: "); Serial.print(gyro_y[1]);
////  Serial.print("  \tGyro z: "); Serial.print(gyro_z[1]);
//  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[1]);
//  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[1]);

//  Serial.print("\nIMU3\t");
////  Serial.print("Gyro x: "); Serial.print(gyro_x[2]);
////  Serial.print("  \tGyro y: "); Serial.print(gyro_y[2]);
////  Serial.print("  \tGyro z: "); Serial.print(gyro_z[2]);
//  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[2]);
//  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[2]);

//  Serial.print("\nIMU4\t");
////  Serial.print("Gyro x: "); Serial.print(gyro_x[3]);
////  Serial.print("  \tGyro y: "); Serial.print(gyro_y[3]);
////  Serial.print("  \tGyro z: "); Serial.print(gyro_z[3]);
//  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[3]);
//  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[3]);

//  Serial.print("\nIMU5\t");
////  Serial.print("Gyro x: "); Serial.print(gyro_x[4]);
////  Serial.print("  \tGyro y: "); Serial.print(gyro_y[4]);
////  Serial.print("  \tGyro z: "); Serial.print(gyro_z[4]);
//  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[4]);
//  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[4]);

  Serial.print("\nIMU6\t");
//  Serial.print("Gyro x: "); Serial.print(gyro_x[5]);
//  Serial.print("  \tGyro y: "); Serial.print(gyro_y[5]);
//  Serial.print("  \tGyro z: "); Serial.print(gyro_z[5]);
  Serial.print("  \tPitch: "); Serial.print(angle_pitch_output[5]);
  Serial.print("  \tRoll: "); Serial.print(angle_roll_output[5]);
  

  while(micros() - loop_timer < 4000);//Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer
}



void computeAngle(int IMU_num){
  
  read_mpu_6050_data(IMU_num);//Read the raw acc and gyro data from the MPU-6050

  gyro_x[IMU_num] -= gyro_x_cal[IMU_num];//Subtract the offset calibration value from the raw gyro_x value
  gyro_y[IMU_num] -= gyro_y_cal[IMU_num];//Subtract the offset calibration value from the raw gyro_y value
  gyro_z[IMU_num] -= gyro_z_cal[IMU_num];//Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch[IMU_num] += gyro_x[IMU_num] * 0.0000611;//Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll[IMU_num] += gyro_y[IMU_num] * 0.0000611;//Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch[IMU_num] += angle_roll[IMU_num]* sin(gyro_z[IMU_num] * 0.000001066);//If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll[IMU_num] -= angle_pitch[IMU_num]* sin(gyro_z[IMU_num] * 0.000001066);//If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector[IMU_num] = sqrt((acc_x[IMU_num]*acc_x[IMU_num])+(acc_y[IMU_num]*acc_y[IMU_num])+(acc_z[IMU_num]*acc_z[IMU_num]));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc[IMU_num] = asin((float)acc_y[IMU_num]/acc_total_vector[IMU_num])* 57.296;//Calculate the pitch angle
  angle_roll_acc[IMU_num] = asin((float)acc_x[IMU_num]/acc_total_vector[IMU_num])* -57.296;//Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc[IMU_num] -= 0.0;//Accelerometer calibration value for pitch
  angle_roll_acc[IMU_num] -= 0.0;//Accelerometer calibration value for roll

  if(set_gyro_angles[IMU_num]){//If the IMU is already started
    angle_pitch[IMU_num] = angle_pitch[IMU_num] * 0.95 + angle_pitch_acc[IMU_num] * 0.05;//Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll[IMU_num] = angle_roll[IMU_num] * 0.95 + angle_roll_acc[IMU_num] * 0.05;//Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{//At first start
    angle_pitch[IMU_num] = angle_pitch_acc[IMU_num];//Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll[IMU_num] = angle_roll_acc[IMU_num];//Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles[IMU_num] = true;//Set the IMU started flag
  }
  
    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output[IMU_num] = angle_pitch_output[IMU_num] * 0.9 + angle_pitch[IMU_num] * 0.1; //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output[IMU_num] = angle_roll_output[IMU_num] * 0.9 + angle_roll[IMU_num] * 0.1;    //Take 90% of the output roll value and add 10% of the raw roll value
}


//Need to input the imu pin number or something to allow us to select the right imu
void read_mpu_6050_data(int IMU_num){//Subroutine for reading the raw gyro and accelerometer data
  int AD0pin = IMU_num + 2;
  digitalWrite(AD0pin,LOW);
  Wire.beginTransmission(MPU_ADDR); //Start communicating with the MPU-6050
  Wire.write(0x3B);                 //Send the requested starting register
  Wire.endTransmission();           //End the transmission
  Wire.requestFrom(MPU_ADDR,14);    //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);     //Wait until all the bytes are received
  acc_x[IMU_num] = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_x variable
  acc_y[IMU_num] = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_y variable
  acc_z[IMU_num] = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_z variable
  temperature[IMU_num] = Wire.read()<<8|Wire.read();  //Add the low and high byte to the temperature variable
  gyro_x[IMU_num] = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_x variable
  gyro_y[IMU_num] = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_y variable
  gyro_z[IMU_num] = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_z variable
  digitalWrite(AD0pin,HIGH);
}


void setup_mpu_6050_registers()
{
  //Activate the MPU-6050 
  Wire.beginTransmission(MPU_ADDR);//Start communicating with the MPU-6050
  Wire.write(0x6B);//Send the requested starting register
  Wire.write(0x00);//Set the requested starting register
  Wire.endTransmission();//End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_ADDR);//Start communicating with the MPU-6050
  Wire.write(0x1C);//Send the requested starting register
  Wire.write(0x10);//Set the requested starting register
  Wire.endTransmission();//End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_ADDR);//Start communicating with the MPU-6050
  Wire.write(0x1B);//Send the requested starting register
  Wire.write(0x08);//Set the requested starting register
  Wire.endTransmission();//End the transmission
}
    
