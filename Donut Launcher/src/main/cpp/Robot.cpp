// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  //m_chooser.SetDefaultOption();
 // m_chooser.AddOption();
  frc::SmartDashboard::PutData("Autos", & m_chooser);
  compressor.EnableDigital();
  lCim.Follow(lNeo);
  rCim.Follow(rNeo);
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  lDriveEncoder.SetPosition(0);
  rDriveEncoder.SetPosition(0);
  //int AutoStage = 0;
  AutoTimer.Start();
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  lDriveEncoder.SetMeasurementPeriod(ROBOTDISTANCEPERPULSE);
	rDriveEncoder.SetMeasurementPeriod(ROBOTDISTANCEPERPULSE);
}

void Robot::TeleopPeriodic() {
  drive.TankDrive(leftDriveStick.GetY(), rightDriveStick.GetY(), false);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(){
  if (!sdfr){
    lDriveEncoder.SetPosition(0);
    rDriveEncoder.SetPosition(0);
    sdfr = true;
  }
  double throttle = (-1 *leftDriveStick.GetY());
  double difference = (-1 * rDriveEncoder.GetPosition()) - (lDriveEncoder.GetPosition());
  drive.TankDrive((throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
  std::cout << "Love isn't always on time" << std::endl; // No I am not ashamed of this TOTO reference
    if (!sdfr){
    lDriveEncoder.SetPosition(0);
    rDriveEncoder.SetPosition(0);
    sdfr = true;
  }
  drive.TankDrive((0.25 * lDriveEncoder.GetPosition()),(-0.25 * rDriveEncoder.GetPosition()), false);
}

void Robot::Abort(){
  auxSpeedController1.StopMotor();
  auxSpeedController2.StopMotor();
  auxSpeedController3.StopMotor();
  auxSpeedController4.StopMotor();
  auxSpeedController5.StopMotor();
  auxSpeedController6.StopMotor();
  pneu1.Set(frc::DoubleSolenoid::Value::kReverse);
  pneu2.Set(frc::DoubleSolenoid::Value::kReverse);
  pneu3.Set(frc::DoubleSolenoid::Value::kReverse);
  pneu4.Set(frc::DoubleSolenoid::Value::kReverse);
  auxSpedCtrlr4DefState = 0;
  auxSpedCtrlr5DefState = 0;
  auxSpedCtrlr6DefState = 0;
  Pnm1DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm2DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm3DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm4DefState = frc::DoubleSolenoid::Value::kReverse;
}

void Robot::Lock(){
  if (gamepad.GetRightBumper()) auxSpedCtrlr4DefState = AUXSPDCTL_SPD;
  if (gamepad.GetLeftBumper()) auxSpedCtrlr5DefState = AUXSPDCTL_SPD;
  if (rightDriveStick.GetTop()) auxSpedCtrlr6DefState = AUXSPDCTL_SPD;
  if (gamepad.GetXButton()) Pnm1DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetYButton()) Pnm2DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetBButton()) Pnm3DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetAButton()) Pnm4DefState = frc::DoubleSolenoid::Value::kForward;
}

int Robot::DistanceDrive (float speed, float distance, bool brake)
{
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
	static float autoStartSpeed;
  static float direction;
	static double lastDistance, speedUpDistance, slowDownDistance;
  static int sameCounter;
  static bool brakingFlag;
  static double brakeStartTime; 

	float newSpeed;
	double curDistance;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    brakingFlag = false;
    FirstCallFlag = false;
    if (speed < 0) {
      direction = -1;
    } else {
      direction = 1;
    }
    autoStartSpeed = direction * AUTOSTARTSPEED;
    if (distance < (DRIVERAMPUPDISTANCE * 2)) {
	    speedUpDistance = distance / 2;
	    slowDownDistance = speedUpDistance;
    } else {
	    speedUpDistance = DRIVERAMPUPDISTANCE;
     	slowDownDistance = distance - DRIVERAMPUPDISTANCE;
    }
	  frc::SmartDashboard::PutNumber(  "DistanceDrive Distance", distance);
  	lastDistance = 0;
    sameCounter = 0;
    lDriveEncoder.SetPosition(0);
  }

 	if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < .2) {
    	drive.TankDrive(-0.2 * direction *FORWARD, -0.2 * direction * FORWARD);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}
  
	curDistance = abs(lDriveEncoder.GetPosition());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((speed - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}

	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
	curDistance = abs(lDriveEncoder.GetPosition());
  if (curDistance < distance) {
    return NOTDONEYET;
  } else {
    if (brake) {
      brakingFlag = true;
      brakeStartTime = (double)AutoTimer.Get();
      return NOTDONEYET;
    } else {
      FirstCallFlag = true;
      drive.TankDrive(0, 0);
      return DONE;
    }
  }
  
  // should never get here
  drive.TankDrive(0, 0);
  FirstCallFlag = true;
  return DONE;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
