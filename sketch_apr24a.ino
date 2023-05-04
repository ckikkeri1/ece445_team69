
#include "BluetoothA2DPSink.h"
#include <ezButton.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BUTTON_PIN 15
ezButton mode_button(BUTTON_PIN);

BluetoothA2DPSink a2dp_sink;

Adafruit_MPU6050 mpu;
float prevVelX = 0;
float VelX = 0;
float temp_accel = 0;
float threshold = 0;

int window_size = 10;
float samples[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sum = 0.0;
int index1 = 0;
float moving_average;
bool first_connection = false;
long lastcheck=0;
bool stationary_mode = 1; 
bool stop_vol = 0;

void setup() {
  // you could also do this also with your own i2s config. But this is simpler
  Serial.begin(115200);    /*Baud rate defined for serial communication*/
 
  i2s_pin_config_t my_pin_config = {
        .bck_io_num = 12,
        .ws_io_num = 14,        
        .data_out_num = 13,
        .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pin_config);

  a2dp_sink.set_bits_per_sample(32);  
  a2dp_sink.start("PCB of life 2");  

  a2dp_sink.set_volume(10);
  
  Serial.println("yo");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  mode_button.setDebounceTime(50);

  setupAccel();
  getThreshold();
  

}

void loop() {
  if(millis() - lastcheck > 300) {
    getAccel();
    lastcheck=millis();
  }
  
  refreshConnection();

  mode_button.loop();
  if (mode_button.isPressed()){
    stationary_mode = (stationary_mode + 1) % 2;
  }
  if (stationary_mode == 0){
    autoVolume();
  }
  
}

void autoVolume(){

  if (moving_average < 5){
    a2dp_sink.set_volume(50);
  }else if (moving_average < 10){
    a2dp_sink.set_volume(75);
  }else if (moving_average < 15){
    a2dp_sink.set_volume(95);
  }else if (moving_average < 20){
    a2dp_sink.set_volume(110);
  }else{
    a2dp_sink.set_volume(120);
  }

}

void refreshConnection(){
  if (a2dp_sink.is_connected()){
    first_connection = true;
    // Serial.println("CONNECTIONNNNNNNNNNNNNNNNNNN");
  }
  if (!a2dp_sink.is_connected() && first_connection){
    ESP.restart();
  }
}

void setupAccel(){
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  // threshold = abs(a.acceleration.y) / 5;
}

void getThreshold(){
  float tsum = 0;
  int first_samples = 30;
  sensors_event_t a, g, temp;
  for (int i = 0; i < first_samples; i++){
    mpu.getEvent(&a, &g, &temp);
    sum += abs(a.acceleration.y) / 5;
    delay(100);
  }
  threshold = sum / first_samples;
  threshold += 0.05;
  Serial.println("-----------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------");
  Serial.print("Threshold: ");
  Serial.println(threshold);
}

// has 200 ms delay
void getAccel(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  temp_accel = abs(a.acceleration.y);
  Serial.print(temp_accel);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  temp_accel = abs(a.acceleration.y) / 5;
  if (temp_accel > threshold)
  {
    // temp_accel = abs(a.acceleration.y) / 5;
    VelX = prevVelX + temp_accel * 0.5;
    VelX = VelX * 2.236 * 5;
    prevVelX = VelX;
  }else{
    // temp_accel = 0;
    // prevVelX = 0;
    VelX = prevVelX / 2;
    prevVelX = VelX;
  }

  if (prevVelX > 2000){
    prevVelX /= 5;
    threshold = threshold + 0.1;
  }

  Serial.print("Our Velocity: ");
  Serial.println(VelX);

  sum += VelX - samples[index1];
  samples[index1] = VelX;
  index1 = (index1 + 1) % window_size;
  moving_average = sum / window_size;

  Serial.print("\nMoving average: ");
  Serial.println(moving_average);
  // delay(500);

}
/*
  Resources used:
    https://github.com/pschatzmann/ESP32-A2DP
    https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
    EZ button library
*/ 