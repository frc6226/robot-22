/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/I2C.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/DutyCycleEncoder.h>
#include <frc/DriverStation.h>
#include <frc/AddressableLED.h>

#include <frc/controller/PIDController.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

typedef struct
{
    double y;
    double z;
    double t;
    double chucker;
    double feeder;
    bool pixy;
    double intake;
} move_step_t;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void set_leds(int r, int g, int b);
  void drive(double y, double z);
  void start_move(move_step_t *move, int count);
  void update_move();

  int get_Pixy_xy(int& x, int& y, int& a, int& colorSignature);
  int get_Pixy_xy(int& x, int& a, int& colorSignature);

  void update_Feeder(double m_feed);

  rev::CANSparkMax m_drive_rm {3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_lm {5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_rf {4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_lf {6, rev::CANSparkMax::MotorType::kBrushless};

  WPI_TalonSRX m_feeder {0};

  frc::DutyCycleEncoder m_dutyCycleEncoder {4};
  frc::DutyCycleEncoder m_winch_encoder {8};

  WPI_TalonSRX m_intake {1};

  rev::CANSparkMax m_chucker{1, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_winch{2, rev::CANSparkMax::MotorType::kBrushless};

  frc::DifferentialDrive m_drive {m_drive_lm, m_drive_rm};

  frc::Joystick m_stick_d {0};
  frc::Joystick m_stick_o {1};

  frc::DigitalInput m_switch_stop {5};
  frc::DigitalInput m_feeder_sw {6};

  //frc::DigitalOutput m_led_ring {9};

  frc::I2C m_pixyPort{frc::I2C::Port::kOnboard, 0x54};
  
  frc::Timer m_timer;
  
  double m_timestamp {0};

  double m_delay;

  bool pixy_drive = false;

  size_t total_steps;
  size_t step;
  double t_step_end;

  bool move_active {0};
  bool step_complete;

  double last_z {0};
  double last_y {0};

  double m_direction {1};
  double accel_max {0.02};

  double m_chucker_speed {0};

  double chucker = 0;
  double intake = 0;

  double m_feeder_sp;

  frc::Counter distanceSensor{frc::Counter::Mode::kSemiperiod};

  static constexpr int kLength = 4; //how many LEDs in the strip
  frc::AddressableLED m_ledStrip{9}; //PWM port
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer; 

  frc::DriverStation::Alliance m_alliance {frc::DriverStation::Alliance::kInvalid};
    
  frc2::PIDController m_feed_pid {0.9, 0, 0};
 
  rev::SparkMaxRelativeEncoder m_encoder_rm = m_drive_rm.GetEncoder();

};