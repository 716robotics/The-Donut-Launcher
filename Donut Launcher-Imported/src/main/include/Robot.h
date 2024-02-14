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
#include <frc/DigitalInput.h>
#include <AHRS.h>
#include <frc/DriverStation.h>
#include <frc/SPI.h>
#include "frc/interfaces/Gyro.h"


class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{1};
  frc::Joystick rightDriveStick{0};
  frc::XboxController gamepad{2};
  //Drive motors
  rev::CANSparkMax rNeo{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax lNeo{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rCim{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax lCim{3, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive drive{lNeo, rNeo};
  //Effectors	
  frc::VictorSP pickup{0};
  frc::VictorSP topShooter{1};
  frc::VictorSP bottomShooter{2};
  frc::VictorSP transporter1{4};
  frc::VictorSP transporter2{3};
  frc::DoubleSolenoid climber{frc::PneumaticsModuleType::CTREPCM, 2, 3,};
  frc::DoubleSolenoid bar4{frc::PneumaticsModuleType::CTREPCM, 0, 1,};
  //Sensors
  rev::SparkRelativeEncoder lDriveEncoder = lNeo.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder rDriveEncoder = rNeo.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
	frc::Encoder shooterEncoder{1,2,false,frc::Encoder::k2X};
  frc::DigitalInput noteCheck{0}; 
  //Global Vars
  frc::Timer AutoTimer;
  bool sdfr = false;
  bool autoactive = true;
  bool tdr = false;
  bool FirstCallFlag = true;
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
  int TurnToAngle(double);

//Vars
  int autoStage;
  int driveDirection = 0;
  bool angleStart;
  double tShotSpeed = 1;
  double bShotSpeed = 1;
  bool moveUp = false;
  int autoType;
  double lDistance;
  double rDistance;
//Auto Chooser
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAuto1dn = "One Donut (Angle)";
  const std::string kAutoDoNothing = "Do Nothing";
  const std::string kAuto2dns = "Two Donuts (Straight)";
  const std::string kAuto2dna = "Two Donuts (Angle)";
  const std::string kAuto3dns = "Three Donuts (Straight)";
  std::string m_autoSelected;

};
AHRS gyro(frc::SPI::Port::kMXP);

