/*
AT+VEL=AAAAAAAA
AT+VEL=AAAAAAA0
AT+VEL=AAA0AAAA
AT+VEL=AA00AA00
*/

#define bldc_l 0
#define bldc_r 1
#define bldc_pwm_r_pin 4
#define bldc_dir_r_pin 5
#define bldc_pwm_l_pin 6
#define bldc_dir_l_pin 7

uint8_t serial_read[1] = {0};

uint8_t read_msg_buf[100] = {0,};
uint8_t vel_msg[17] = "AT+VEL=00000000\n\0";
uint8_t read_msg_len = 100;
uint8_t read_msg_cnt = 0;

float vel_lin = 0;
float vel_ang = 0;
float vel_l = 0;
float vel_r = 0;
uint8_t vel_l_pwm = 0;
uint8_t vel_r_pwm = 0;

#define debug_
void setup() 
{
  Serial.begin(9600);

  pinMode(bldc_pwm_l_pin,OUTPUT);
  pinMode(bldc_dir_l_pin,OUTPUT);
  pinMode(bldc_pwm_r_pin,OUTPUT);
  pinMode(bldc_dir_r_pin,OUTPUT);

#ifdef debug_
  Serial.print("==== ================ ====\n");
  Serial.print("Jetson Serial Communicator\n");
  Serial.print("==== ================ ====\n");
#endif

}

void loop() 
{
  if(Serial.available())
  {
    serial_read[0] = Serial.read();
    CheckData(serial_read[0]);
  }
}

void BLDCCtrl(uint8_t motor_id, float motor_pwm)
{
  bool motor_dir_ = LOW;
  float motor_pwm_abs_ = 0;

  if(motor_pwm > 0)
  {
    motor_pwm_abs_ = motor_pwm;
  }
  else
  {
    motor_pwm_abs_ = -motor_pwm;
    motor_dir_ = HIGH;
  }

  if(motor_id == bldc_l)
  {
    analogWrite(bldc_pwm_l_pin, motor_pwm_abs_);
    digitalWrite(bldc_dir_l_pin,motor_dir_);
  }
  else if(motor_id == bldc_r)
  {
    analogWrite(bldc_pwm_r_pin, motor_pwm_abs_);
    digitalWrite(bldc_dir_r_pin,motor_dir_);

  }
}


// ==== function ====//
bool CheckData(uint8_t input_)
{
  bool res_ = false;
  read_msg_buf[read_msg_cnt] = input_;
  read_msg_cnt++;

  if(input_ == '\n')
  {
    if(CheckArray(vel_msg, read_msg_buf, 7)==true && read_msg_cnt==16)
    {
      res_ = true;
      memcpy(&vel_lin, &read_msg_buf[7], 4);
      memcpy(&vel_ang, &read_msg_buf[11], 4);

      //vel_lin = vel_r+vel_l
      //vel_ang = vel_r-vel_l

      vel_l = 0.5*(vel_lin+vel_ang);
      vel_r = 0.5*(vel_lin-vel_ang);
      vel_l_pwm=50*vel_l;
      vel_r_pwm=50*vel_r;
      
      if(vel_l_pwm > 255) vel_l_pwm = 255;
      else if(vel_l_pwm < -255) vel_l_pwm = -255;

      if(vel_r_pwm > 255) vel_r_pwm = 255;
      else if(vel_r_pwm < -255) vel_r_pwm = -255;

      BLDCCtrl(bldc_l, vel_l_pwm);
      BLDCCtrl(bldc_r, vel_r_pwm);
      
      memcpy(vel_msg, read_msg_buf, read_msg_cnt);

#ifdef debug_
      Serial.print("\nByte read : ");
      for(uint8_t j=0;j<read_msg_cnt;j++)
      {
      Serial.print(read_msg_buf[j]);
      Serial.print(", ");
      }

      Serial.print("\nByte Vel Lin : ");
      for(uint8_t j=7;j<11;j++)
      {
      Serial.print(read_msg_buf[j]);
      Serial.print(", ");
      }
      Serial.print(" Vel Ang : ");
      for(uint8_t j=11;j<15;j++)
      {
      Serial.print(read_msg_buf[j]);
      Serial.print(", ");
      }

      Serial.print("\nRobot vel Lin : ");
      Serial.print(vel_lin);
      Serial.print(", Ang : ");
      Serial.println(vel_ang);
      Serial.print("Robot vel Left : ");
      Serial.print(vel_l);
      Serial.print(", right : ");
      Serial.print(vel_r);
      Serial.print(", PWM Left : ");
      Serial.print(vel_l_pwm);
      Serial.print(", Right : ");
      Serial.println(vel_r_pwm);
#endif
    }
    else
    {
      Serial.println(read_msg_cnt);
    }

    memset(read_msg_buf, 0, read_msg_len);
    read_msg_cnt = 0;

  }

  return res_;
}

bool CheckArray(uint8_t msg_target_[], uint8_t msg_input_[],uint16_t msg_len_)
{
  for(uint8_t i=0;i<msg_len_;i++)
  {
    if(msg_target_[i] != msg_input_[i])
    {
      return false;
    }
  }
  return true;
}