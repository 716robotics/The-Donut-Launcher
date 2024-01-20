// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Approximate distance for auto to drive forward in inches
#define AUTODIST 500

#pragma once
#include "tunables.h"
#include <string>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Relay.h>
#include <frc/Servo.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/Can.h>
#include <rev/CANSparkMax.h> 


class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{0};
  frc::Joystick rightDriveStick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  rev::CANSparkMax rNeo{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax lNeo{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rCim{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax lCim{3, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive drive{lNeo, rNeo};
  //Effectors
  frc::Compressor compressor;
  frc::VictorSP auxSpeedController1{0};
  frc::VictorSP auxSpeedController2{1};
  frc::VictorSP auxSpeedController3{2};
  frc::VictorSP auxSpeedController4{3};
  frc::VictorSP auxSpeedController5{4};
  frc::VictorSP auxSpeedController6{5};
  frc::DoubleSolenoid pneu1{frc::PneumaticsModuleType::CTREPCM, 0, 1,};
  frc::DoubleSolenoid pneu2{frc::PneumaticsModuleType::CTREPCM, 2, 3,};
  frc::DoubleSolenoid pneu3{frc::PneumaticsModuleType::CTREPCM, 4, 5,};
  frc::DoubleSolenoid pneu4{frc::PneumaticsModuleType::CTREPCM, 6, 7,};
  //Sensors
  rev::SparkRelativeEncoder lDriveEncoder = lNeo.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 4096);
  rev::SparkRelativeEncoder rDriveEncoder = rNeo.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 4096);
	//frc::Encoder leftDriveEncoder{0,1,false,frc::Encoder::k4X};
	//frc::Encoder rightDriveEncoder{2,3,false,frc::Encoder::k4X};
  //Global Vars
  frc::Timer AutoTimer;
  bool sdfr = false;
  bool autoactive = true;
  //Default States
  float auxSpedCtrlr4DefState = 0;
  float auxSpedCtrlr5DefState = 0;
  float auxSpedCtrlr6DefState = 0;
  frc::DoubleSolenoid::Value Pnm1DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm2DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm3DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm4DefState = frc::DoubleSolenoid::Value::kReverse;
//Functions
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void StraightDrive();
  void HoldTheLine();
  void Abort();
  void Lock();
  int DistanceDrive(float,float,bool);

  //Vars
int autoStage;

//Auto Chooser
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoDriveForward = "Drive Forward";
  const std::string kAutoDoNothing = "Do Nothing";
  std::string m_autoSelected;
};
