/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#define SIZEOF_ARRAY(array_name) (sizeof(array_name) / sizeof(array_name[0]))

const char* version = "v22.3.4";

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

move_step_t mv_default[] =
{
  // delay
  // {0.0, 0.0, 5.0, 0.68, 0.0, 0, 0.0, 0.0},
  //{drive,steer,time,chucker,feeder,pixy(bool), intake, position_rm}
  {0.15, 0.0, 0.7, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.4, 0, 0.0},
  {0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0},
 
  // pixy drive
  {0.15, 0.0, 2.5, 0.0, 0.0, 1, -0.7},

  {0.0, 0.0, 0.5, -0.65, 0.0, 0, -0.7},

  //LL drive
  {-0.15, 0.0, 2.8, -0.65, 0.0, 1, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1, -0.65, 0.4, 0, 0.0},
  // stop
  {0.0, 0.0, 2.5, 0.0, 0.0, 0, 0.0},
};


move_step_t mv_auto_2[] =
{
  // delay
  // {0.0, 0.0, 5.0, 0.68, 0.0, 0, 0.0, 0.0},
  //{drive,steer,time,chucker,feeder,pixy(bool), intake, position_rm}
  {0.15, 0.0, 0.7, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.4, 0, 0.0},
  {0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0},
 
  // pixy drive
  {0.15, 0.0, 2.5, 0.0, 0.0, 0, -0.7},

  {0.0, 0.0, 0.5, -0.65, 0.0, 0, -0.7},

  //LL drive
  {-0.15, 0.0, 2.8, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1.0, -0.65, 0.0, 0, 0.0},
  {0.0, 0.0, 1, -0.65, 0.4, 0, 0.0},
  // stop
  {0.0, 0.0, 2.5, 0.0, 0.0, 0, 0.0},
};

move_step_t mv_auto_3[] =
{
  // delay
  // {0.0, 0.0, 5.0, 0.68, 0.0, 0, 0.0},
  //{drive,steer,time,chucker,feeder,pixy(bool),intake,position_rm}
  {0.25, 0.0, 0.5, -0.65, 0.0, 0, 0.0}, //back up a little
  {0.0, 0.0, 1.5, -0.65, 0.0, 1, 0.0},  //use the limelight
  {0.0, 0.0, 0.5, -0.65, 0.0, 0, 0.0},  //wait half a sec
  {0.0, 0.0, 1.0, -0.65, 0.4, 0, 0.0},  //shoot
  {0.0, 0.0, 0.25, 0.0, 0.0, 0, 0.0},   //wait quarter sec

  // pixy drive
  {0.25, 0.0, 3.0, 0.0, 0.0, 1, -0.7}, //drive rev pixy and intake on

  {0.0, 0.0, 0.5, 0.0, 0.0, 0, -0.7}, //wait half sec with intake on
  {-0.25, 0.0, 3.0, -0.65, 0.0, 1, 0.0},  //drive slow with limelight and shooter on
  {0.0, 0.0, 1.0, -0.65, 0.0, 1, 0.0},  //aim with LL
  {0.0, 0.0, 0.5, -0.65, 0.0, 0, 0.0},  //wait half sec
  {0.0, 0.0, 1.0, -0.65, 0.4, 0, 0.0},  //shoot
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0},  //stop
};
//{drive,steer,time,chucker,feeder,pixy(bool), intake}

move_step_t none[] =
{
  {0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0},
};

move_seq_t mv = {none, 0};

#if 1
static void clamp(double &value, const double ul, const double ll)
{
    if (value < ll)
  {
      value = ll;
  }
    else if (value > ul)
  {
      value = ul;
  }
}
#endif

#if 1
static void scale(double &value, const double deadband, const double ll, const double ul)
{
  bool positive = (value >= 0.0);

  if (!positive) value *= -1;

//printf("1: value=%5.2f\n", value);

  // adjust for deadband
  if (value > deadband) value -= deadband;
  // else if (value < -deadband) value += deadband;
  else value = 0;

//printf("2: value=%5.2f\n", value);

  // scale based output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

//printf("3: value=%5.2f\n", value);

  // offset to minimum
  value += ll;
  // if (value > 0) value += ll;
  // else value -= ll;

  if (!positive) value *= -1;

//printf("4: value=%5.2f\n", value);
}
#endif



void Robot::RobotInit()
{
    printf("%s: %s %s\n", version, __DATE__, __TIME__);

    m_timer.Reset();
    m_timer.Start();

    m_dutyCycleEncoder.SetDistancePerRotation(0.5);

    m_drive_rm.RestoreFactoryDefaults();
    m_drive_rf.RestoreFactoryDefaults();
    m_drive_lm.RestoreFactoryDefaults();
    m_drive_lf.RestoreFactoryDefaults();

    /* set up followers */
    m_drive_rf.Follow(m_drive_rm);
    m_drive_lf.Follow(m_drive_lm);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    m_drive_rm.SetInverted(true);
    m_drive_rf.SetInverted(true);
    m_drive_lm.SetInverted(false);
    m_drive_lf.SetInverted(false);

    /* [4] adjust sensor phase so sensor moves
        * positive when Talon LEDs are green */
    //m_drive_rm.SetSensorPhase(true);
    //m_drive_lm.SetSensorPhase(false);

    m_drive_rm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_drive_rf.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_drive_lm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_drive_lf.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_winch.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_feeder.SetNeutralMode(Brake);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);

#if 0
    frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
#endif

    frc::SmartDashboard::PutNumber("delay", 0);
    frc::SmartDashboard::PutNumber("fw_sp1", 0.2);
    frc::SmartDashboard::PutNumber("auto", 1);

    m_alliance = frc::DriverStation::GetAlliance();

    printf("%s alliance\n", m_alliance == frc::DriverStation::Alliance::kRed ? "red" : "blue");

    distanceSensor.SetUpSource(9);
    distanceSensor.SetSemiPeriodMode(true);
    distanceSensor.SetSamplesToAverage(2);

    m_ledStrip.SetLength(kLength);
    m_ledStrip.SetData(m_ledBuffer);
    m_ledStrip.Start();

}

void Robot::AutonomousInit()
{
  m_delay = frc::SmartDashboard::GetNumber("delay", 1);

  m_chucker_speed = frc::SmartDashboard::GetNumber("fw_sp1", .4);

  // normalize the setting
  while (m_chucker_speed < -1.0 || m_chucker_speed > 1.0)
    m_chucker_speed /= 10;

  // determine the auto routine to use
  int auto_selection = frc::SmartDashboard::GetNumber("auto", 1);
  printf("auto = %d\n", auto_selection);

  if (auto_selection == 1)
  {
    // mv_default[0].t = m_delay;
    start_move(mv_default, SIZEOF_ARRAY(mv_default));
  }
  else if (auto_selection == 2)
  {
    // mv_auto_2[0].t = m_delay;
    start_move(mv_auto_2, SIZEOF_ARRAY(mv_auto_2));
  }
  else if (auto_selection == 3)
  {
    // mv_auto_3[0].t = m_delay;
    start_move(mv_auto_3, SIZEOF_ARRAY(mv_auto_3));
  }
  else
  {
    mv_default[0].t = m_delay;
    start_move(mv_default, SIZEOF_ARRAY(mv_default));
  }
  
  m_alliance = frc::DriverStation::GetAlliance();

  printf("%s alliance\n", m_alliance == frc::DriverStation::Alliance::kRed ? "red" : "blue");

  m_dutyCycleEncoder.Reset();

  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
}

void Robot::AutonomousPeriodic()
{
  update_move();

  double y = 0.0;
  double z = 0.0;

  pixy_drive = false;

  //{ 
  //  double winch = 0;
  //  if (!m_switch_stop.Get())
  //  {
  //    winch = -0.1;
  //  }
  //  m_winch.Set(winch);
  //}
  
  if (step < mv.total_steps)
  {
    y = mv.steps[step].y;
    z = mv.steps[step].z;

    pixy_drive = mv.steps[step].pixy;
  }

  if (y == 0.0)
  {
      // set sign for pure rotations
      // z *= rotation;
  }
    //driving back for cargo. use the pixy
  if (pixy_drive && y > 0.00) 
  {
    int px;
    int pa;   //number of frames the camera maitained the target (age)
    int color = m_alliance;
    
    get_Pixy_xy(px, pa, color);

    // printf("x=%d y=%d pa=%d color=%d\n", px, py, pa, color);

    if (color != 1 && color != 2)
    {
    }
    else if(px > -160 && px < 160 && pa > 10)        //a valid target is in the camera frame
    {
      //printf("target=%d %d\n" , xDirectionValue, yDirectionValue);    //since both values seem valid, print them
      printf("Target acquired\n");
      y = .15;
      z = ((double)px / 60) * 0.27;
      printf("px=%d z=%5.2f\n", px, z);
    }
    
    else
    {
      printf("No Target acquired\n");
      y = .15;
      z = 0;
    }
  }
  // driving forward. Use the Limelight
  if(pixy_drive && y <= 0.0)
  {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
    double Kp = 0.01;  //turn down if oscillating
    double min_command = 0.04;
    double steering_adjust = 0.0;
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
    double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);

    if(tv){
      if(tx > 0.8)
      {
        steering_adjust = Kp*tx + min_command;
        z = steering_adjust;
      }
      if(tx < -0.8)
      {
        steering_adjust = Kp*tx - min_command;
        z = steering_adjust;
      }
    }
  }

  drive(y, z);

  m_chucker.Set(mv.steps[step].chucker);
  m_intake.Set(mv.steps[step].intake);

  m_feeder_sp = (mv.steps[step].feeder);
  update_Feeder(m_feeder_sp);
}

void Robot::TeleopInit()
{
  m_chucker_speed = frc::SmartDashboard::GetNumber("fw_sp1", .4);

  // normalize the setting
  while (m_chucker_speed < -1.0 || m_chucker_speed > 1.0)
    m_chucker_speed /= 10;

  m_drive_rm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_rf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_lm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_lf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  m_timestamp = 0;
  move_active = false; // no active move

  chucker = 0;
  intake = 0;
  m_feeder_sp = 0;

  m_alliance = frc::DriverStation::GetAlliance();
  printf("alliance=%d\n", m_alliance);

  m_dutyCycleEncoder.Reset();
  m_winch_encoder.Reset();

}

void Robot::TeleopPeriodic()
{
  double winch = 0;
  set_leds(0, 0, 0);
  // driver input
  double y = m_stick_d.GetRawAxis(1) * m_direction;
  double z = m_stick_d.GetRawAxis(4);

#if 1
  // carpet
  scale(y, 0.15, 0.0, .60);
  scale(z, 0.15, 0.0, .36);    
#else
  // tile
  //scale(y, 0.15, 0.0, 0.45);
  //scale(z, 0.15, 0.0, .25); 
  // slow demo
  scale(y, 0.15, 0.0, 0.25);
  scale(z, 0.15, 0.0, .15);
#endif

  if (m_stick_o.GetRawButtonPressed(4))
  {
    chucker = -0.66;
    //chucker = -0.55;
  }

  if (m_stick_d.GetRawButton(6))
  {
    m_feeder_sp = 0.4;
  }
  else
  {
    m_feeder_sp = 0;
  }

  update_Feeder(m_feeder_sp);

  if (m_stick_o.GetRawButtonPressed(3))
  {
    intake = -0.70;
  }

  if (m_stick_o.GetRawButtonPressed(2))
  {
    intake = 0.0;
    chucker = 0.0;
  }

  double winch_pos = m_winch_encoder.Get().value();
   printf("winch = %5.2f\n", winch_pos);

  if (m_stick_o.GetRawButton(7) && winch_pos < 1.9)
  {
    winch = (1.0);
  }
  else if (m_stick_o.GetRawButton(8) && !m_switch_stop.Get())
  {
    winch = (-1.0);
    m_winch_encoder.Reset();
  }
  
  if (!m_switch_stop.Get())
  {
    set_leds(255, 0, 0);
    clamp(y, 0.15, -0.15);
    clamp(z, 0.12, -0.12);
  }


  if (m_stick_d.GetRawButton(1))
  {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0); //light on
    if(y > 0.00) //Driving in reverse so use the Pixy
    {
      int px, pa, color = m_alliance;
      get_Pixy_xy(px, pa, color);
      printf("px=%d pa=%d color=%d\n", px, pa, color);

      if (color != 1 && color != 2) //blue or red not detected
      {
      } 
      else if(px > -160 && px < 160 && pa > 10)
      {
        // target found
        // z = ((double)px / 60) * 0.24;
        set_leds(0, 0, 255);
        z = ((double)px / 60) * 0.25;
        printf("px=%d z=%5.2f\n", px, z);
      }
      else
      {
        z = 0;
      }
    }
    else //driving forward or not driving so use the Limelight
    {
      double Kp = 0.01;  //turn down if oscillating
      double min_command = 0.04;
      double steering_adjust = 0.0;
      double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);

      if(tv)
      {
        if(tx > 0.8)
        {
          steering_adjust = Kp*tx + min_command;
          z = steering_adjust;
        }
        if(tx < -0.8)
        {
          steering_adjust = Kp*tx - min_command;
          z = steering_adjust;
        }
      //printf("z=%5.2f tx=%5.2f steering_adjust=%5.2f\n", z, tx, steering_adjust);
      }
    }
  }
  else
  {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);  //light off
  }
  // update the motors every pass
  drive(y, z);


  m_intake.Set(intake);
  m_chucker.Set(chucker);
  m_winch.Set(winch);

  // ultrasonic ranging
  auto distance = distanceSensor.GetPeriod().value() * 1000;    // 1 uSec = 1 mm
  printf("d = %5.2f\n", distance);

  if(distance < 0.55 )
  {
    set_leds(0, 255, 0);
  }
}

void Robot::DisabledInit() 
{
  m_drive_rm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_rf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_lm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_drive_lf.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_winch.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
}
void Robot::TestPeriodic() {}

void Robot::set_leds(int r, int g, int b)
{
    for(unsigned int i = 0; i < m_ledBuffer.size(); i++)
    {
      m_ledBuffer[i].SetRGB(r, g, b);
    }
    m_ledStrip.SetData(m_ledBuffer);
}

void Robot::drive(double y, double z)
{
  double tmp_y = last_y;
  double tmp_z = last_z;

  // limit acceleration
  double dy = y - last_y;
  double dz = z - last_z;

  clamp(dy, accel_max, -accel_max);
  clamp(dz, accel_max, -accel_max);

  last_y = last_y + dy;
  last_z = last_z + dz;

  if (tmp_y != last_y || tmp_z != last_z)
  {
    // printf("y: %5.2f => %5.2f / z: %5.2f => %5.2f\n", y, last_y, z, last_z);
  }

  // if(pixy_drive)
  // {
  //   last_z = z;
  // }

  m_drive.ArcadeDrive(-last_y, last_z, false);
}

void Robot::start_move(move_step_t *move, int count)
{
  mv.steps = move;
  mv.total_steps = count;

  auto tv = m_timer.Get();

  // t_step_end = m_timer.Get() + m_delay;
  //t_step_end = m_timer.Get() + mv.steps[0].t;
  t_step_end = tv.value() + mv.steps[0].t;
  step = 0;

  move_active = true;

  printf("%d: y=%5.2f z=%5.2f t=%5.2f l=%5.2f f=%5.2f\n", step+1,
      mv.steps[0].y, mv.steps[0].z, mv.steps[0].t, mv.steps[0].chucker, mv.steps[0].feeder);
}


void Robot::update_move()
{
  if (!move_active) return;

  auto tv = m_timer.Get();

  printf("t=%5.2f end=%5.2f\n", tv.value(), t_step_end);
  if (step < mv.total_steps)
  {
      step_complete = tv.value() > t_step_end;

      if (step_complete)
      {
        step += 1;
          if (step < mv.total_steps)
          {
              t_step_end += mv.steps[step].t;

              printf("%d: y=%5.2f z=%5.2f t=%5.2f l=%5.2f f=%5.2f, p=%d, i=%5.2f\n", step+1,
                  mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].chucker, mv.steps[step].feeder,
                  mv.steps[step].pixy, mv.steps[step].intake);
          }
          else
          {
              // sequence complete
            printf("%5.2f: move complete\n", tv.value());

            step = mv.total_steps;
            move_active = false;
          }
      }
  }
}


#if 0
int Robot::get_Pixy_xy(int& x, int& y, int& a, int& colorSignature)
{
  //Pixy Camera:
  //sendPacket byte 0-1 16-bit sync, 2 type, 3 length of payload (len), 4-len payload
  //recvPacket byte 0-1 16-bit sync(174, 193), 2 type, 3 length of payload (len), 4-5 16-bit checksum, 6-len payload
  //byte 18 of the receiveBuffer is the tracking index. the nuber assigned to a newly found block.
  //byte 19 of the receiveBuffer is the age in frames. when it reaches 255 it stops tracking.

  //uint8_t setLamp[]={174,193,22,2,0,0};   //Turn off all LEDs including RGB
  //uint8_t setLamp[]={174,193,22,2,1,0};   //Turn on top LEDs
  //m_pixyPort.WriteBulk(setLamp, 6);

  uint8_t sig = colorSignature == 0 ? 1 : 2;
  uint8_t receiveBufffer[32];
  uint8_t getBlocks[]={174,193,32,2,sig,1};   //from the Pixy2 data sheet (sigmap 1,max blocks to return 1)
  m_pixyPort.WriteBulk(getBlocks, 6);       //from the frc::I2C class
  m_pixyPort.ReadOnly(32, receiveBufffer);  //from the frc::I2C class
  int xDirectionValue = receiveBufffer[8] | receiveBufffer[9]<<8;   //using two bytes. max is 315
  int yDirectionValue = receiveBufffer[10];             //max is 207. only need one byte.
  colorSignature = receiveBufffer[6];

  a = receiveBufffer[19];
  x = xDirectionValue - 160;
  if(pixy_drive)
  {
    y = yDirectionValue; //only update y in autonomous when pixy_drive is true
  }
  else
  {
    y = 0;
  }
  //frc::SmartDashboard::PutString("DB/String 5", "Index: "+ std::to_string(receiveBufffer[18]));
  //frc::SmartDashboard::PutString("DB/String 6", "Age: "+ std::to_string(a));

  return true;
}
#endif

int Robot::get_Pixy_xy(int& x, int& a, int& colorSignature)
{
  //Pixy Camera:
  //sendPacket byte 0-1 16-bit sync, 2 type, 3 length of payload (len), 4-len payload
  //recvPacket byte 0-1 16-bit sync(174, 193), 2 type, 3 length of payload (len), 4-5 16-bit checksum, 6-len payload
  //byte 18 of the receiveBuffer is the tracking index. the nuber assigned to a newly found block.
  //byte 19 of the receiveBuffer is the age in frames. when it reaches 255 it stops tracking.

  //uint8_t setLamp[]={174,193,22,2,0,0};   //Turn off all LEDs including RGB
  //uint8_t setLamp[]={174,193,22,2,1,0};   //Turn on top LEDs
  //m_pixyPort.WriteBulk(setLamp, 6);

  uint8_t sig = colorSignature == 0 ? 1 : 2;
  uint8_t receiveBufffer[32];
  uint8_t getBlocks[]={174,193,32,2,sig,1};   //from the Pixy2 data sheet (sigmap 1,max blocks to return 1)
  m_pixyPort.WriteBulk(getBlocks, 6);       //from the frc::I2C class
  m_pixyPort.ReadOnly(32, receiveBufffer);  //from the frc::I2C class
  int xDirectionValue = receiveBufffer[8] | receiveBufffer[9]<<8;   //using two bytes. max is 315
  colorSignature = receiveBufffer[6];
  
  a = receiveBufffer[19];
  x = xDirectionValue - 160;

  // frc::SmartDashboard::PutString("DB/String 5", "Index: "+ std::to_string(receiveBufffer[18]));
  // frc::SmartDashboard::PutString("DB/String 6", "Age: "+ std::to_string(a));

  return true;
}

void Robot::update_Feeder(double m_feed)
{
  double feeder = 0;
  auto output = m_dutyCycleEncoder.Get();
  bool feeder_sw = m_feeder_sw.Get();

  if (m_feed > 0)
  {
    feeder = m_feed_pid.Calculate(output.value(), m_feed) * -1;
  }
  else
  {
    if (feeder_sw) 
      feeder = 0.20;
    else 
      m_dutyCycleEncoder.Reset();
  }
  clamp(feeder, 0.20,-0.35);
  // printf("feed = %5.2f\n", feeder);
  m_feeder.Set(feeder);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
 