// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  // Auto Types
  m_chooser.SetDefaultOption(kAutoDoNothing,kAutoDoNothing);
  m_chooser.AddOption(kAuto1dn,kAuto1dn);
  m_chooser.AddOption(kAuto2dns,kAuto2dns);
  m_chooser.AddOption(kAuto2dna,kAuto2dna);
  m_chooser.AddOption(kAuto3dns,kAuto3dns);
  frc::SmartDashboard::PutData("Autos", & m_chooser);
  shooterEncoder.SetDistancePerPulse(0.01752);//should give speed in rpm
  lDriveEncoder.SetMeasurementPeriod(ROBOTDISTANCEPERPULSE);
	rDriveEncoder.SetMeasurementPeriod(ROBOTDISTANCEPERPULSE);

  //Cim following
    lCim.Follow(lNeo, true);
    rCim.Follow(rNeo, true);
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Direction var", driveDirection);
  frc::SmartDashboard::PutNumber("Left Drive Encoder", lDistance);
  frc::SmartDashboard::PutNumber("Right Drive Encoder", rDistance);
  frc::SmartDashboard::PutBoolean("Note Checker", noteCheck.Get());
  frc::SmartDashboard::PutNumber("Shot Encoder", shooterEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Shooter Speed", shooterEncoder.GetRate());
  frc::SmartDashboard::PutNumber("Gyro Yaw", gyro.GetYaw());
  frc::SmartDashboard::PutNumber("Gyro Pitch", gyro.GetPitch());
  frc::SmartDashboard::PutNumber("Gyro Roll", gyro.GetRoll());
  frc::SmartDashboard::PutNumber("tShot Projected Speed",tShotSpeed);
  frc::SmartDashboard::PutNumber("bShot Projected Speed",bShotSpeed);
  frc::SmartDashboard::PutNumber("D-Pad Value", gamepad.GetPOV());
  frc::SmartDashboard::PutNumber("Auto Timer", (double)AutoTimer.Get());
  lDistance = lDriveEncoder.GetPosition() * 1.945;
  rDistance = rDriveEncoder.GetPosition() * 1.945;
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if(m_autoSelected == kAuto1dn){
    autoType = 1;
  }
  else if(m_autoSelected == kAuto2dns){
    autoType = 2;
  }
  else if(m_autoSelected == kAuto2dna){
    autoType = 3;
  }
  else if(m_autoSelected == kAuto3dns){
    autoType = 4;
  }
  else{
    autoType = 0;
  }

  AutoTimer.Start();
}

void Robot::AutonomousPeriodic() {
  switch (autoType) {

    //Auto Do Nothing
    case 0:
    lNeo.Set(0);
    rNeo.Set(0);
    pickup.Set(0);
    transporter1.Set(0);
    transporter2.Set(0);
    topShooter.Set(0);
    bottomShooter.Set(0);
    bar4.Set(bar4.kOff);
    climber.Set(climber.kOff);
    break;

    //Auto 1 Donut
    case 1:
    switch(autoStage){
      //Turn on Shooters
      case 0:
      if(shooterEncoder.GetRate() <= -600){
        autoStage = 1;
      }
        topShooter.Set(-tShotSpeed);
        bottomShooter.Set(bShotSpeed);
      break;
      //Fire + Turn off
      case 1:
      if(!noteCheck.Get()){
        topShooter.Set(0);
        bottomShooter.Set(0);
        transporter2.Set(0);
        autoStage = 2;
      }
      else{
         transporter2.Set(.3);
      }
      break;
      //Mobility
      case 2:
      if(DistanceDrive(-.2,30,true) == DONE){
        drive.TankDrive(0,0,false);
        autoStage = 3;
        sdfr = false;
      }
      else{
        DistanceDrive(-.2,30,true);
      }
      break;
      //Finish
      case 3:
      autoType = 0;
      break;
    }
    break;

    //Auto 2 Donut (Straight)
    case 2:
    switch (autoStage){
      //Turn on Shooters
      case 0:
      if(shooterEncoder.GetRate() <= -600){
        autoStage = 1;
      }
        topShooter.Set(-tShotSpeed);
        bottomShooter.Set(bShotSpeed);
      break;
      //Fire + Turn off
      case 1:
      if(!noteCheck.Get()){
        topShooter.Set(0);
        bottomShooter.Set(0);
        transporter2.Set(0);
        autoStage = 2;
        lDriveEncoder.SetPosition(0);
        rDriveEncoder.SetPosition(0);
        AutoTimer.Reset();
      }
      else{
         transporter2.Set(.3);
      }
      break;

      //Pickup Down
      case 2:
      bar4.Set(bar4.kForward);
      if((double)AutoTimer.Get() >= 1){
      autoStage = 3;
      }
      break;
      case 3:
      
      if(DistanceDrive(-.2,30,true) == DONE){
        printf("Case 3: Done driving ")
        drive.TankDrive(0,0,true);
        if(noteCheck.Get()){
          pickup.Set(0);
          transporter1.Set(0);
          transporter2.Set(0);
          bar4.Set(bar4.kReverse);
          sdfr = false;
          autoStage = 5;
          printf("moving to stage 5, encoders: (%f, %f)", lDistance, rDistance);
          AutoTimer.Reset();
        }
      }
      else if(lDistance <= -10){
        pickup.Set(.7);
        transporter1.Set(.5);
        transporter2.Set(.3);
      }
      printf("\n");
      break;
      
      //Drive Back
      case 5:
      if((double)AutoTimer.Get() >= 3){
        printf(".");
        if(DistanceDrive(-.2, 30, true) == DONE){
          printf("\nStage 5 driving complete (%f, %f) after %f seconds.\n", lDistance, rDistance, (float)AutoTimer.Get());
          drive.TankDrive(0,0,true);
          autoStage = 6;
          printf("Moving to stage 6\n");
          sdfr = false;
        }
      }
      break;

      //Shoot
      case 6:
      if(shooterEncoder.GetRate() <= -600){
        autoStage = 7;
      }
        topShooter.Set(-tShotSpeed);
        bottomShooter.Set(bShotSpeed);
      break;
      case 7:
      if(!noteCheck.Get()){
        topShooter.Set(0);
        bottomShooter.Set(0);
        transporter2.Set(0);
        autoStage = 8;
      }
      else{
         transporter2.Set(.3);
      }

      break;

      //Finish
      case 8:
      autoType = 0;
      break;

      break;
    } 

    break;

    //Auto 2 Donut (Angle)
    case 3:
    switch(autoStage){
      //Turn on Shooters
      case 0:
      if(shooterEncoder.GetRate() <= -600){
        autoStage = 1;
      }
        topShooter.Set(-tShotSpeed);
        bottomShooter.Set(bShotSpeed);
      break;
      autoStage = 1;

    break;
    case 1:
    if(DistanceDrive(-.2, 20, true) == DONE){
      drive.TankDrive(0,0,true);
      autoStage = 2;
      sdfr = false;
      }
     break;
     case 2:
     if(TurnToAngle(-45) == DONE){
      drive.TankDrive(0,0,true);
      autoStage = 3;
      tdr = false;
     }
     break;
     case 3:
     if(DistanceDrive(-.2, 20, true) == DONE){
      drive.TankDrive(0,0,true);
      autoStage = 4;
      sdfr = false;
      AutoTimer.Reset();
     }
     break;
     case 4:
      bar4.Set(bar4.kForward);
      if((double)AutoTimer.Get() >= 1){
      autoStage = 5;
      }
     break;
     case 5:
     if(DistanceDrive(-.2,30,true) == DONE){
        drive.TankDrive(0,0,true);
        if(noteCheck.Get()){
        pickup.Set(0);
        transporter1.Set(0);
        transporter2.Set(0);
        bar4.Set(bar4.kReverse);
        sdfr = false;
        autoStage = 6;
        }
      }
      else{
      if(lDistance <= -10){
        pickup.Set(.7);
        transporter1.Set(.5);
        transporter2.Set(.3);
      }
      }

     break;
    }

    //Auto 3 Donut (Straight)
    case 4:
    break;

  

 }
}

void Robot::TeleopInit() {
  driveDirection = 0;
}

void Robot::TeleopPeriodic() {
  //reverse drive function
  if(leftDriveStick.GetRawButton(2)){
    driveDirection = 0;
  }
  else if(rightDriveStick.GetRawButton(3)){
    driveDirection = 1;
  }
  
  else if(driveDirection == 1){
    drive.TankDrive(leftDriveStick.GetY(),rightDriveStick.GetY() * -1, false);
  }
  else if(driveDirection == 0){
    drive.TankDrive(rightDriveStick.GetY() * - 1,leftDriveStick.GetY(), false);
  }
  

  // Waiting For Sensor
  if(gamepad.GetXButton()){
    moveUp = true;
  }

  // Pickup Up (Override)
  if(gamepad.GetBButton()){
    bar4.Set(bar4.kReverse);
    moveUp = false;
  }
  // Pickup Up (Sensor)
  else if(moveUp && noteCheck.Get()){
      bar4.Set(bar4.kReverse);
      moveUp = false;
  }
  // Pickup Down
  else if(gamepad.GetAButton()){
    bar4.Set(bar4.kForward);
    moveUp = false;
  }
  else{
    bar4.Set(bar4.kOff);
  }

  // Pickup Wheels + Transporter Wheels
  if(gamepad.GetRightTriggerAxis() >= .7){
    transporter1.Set(1);
    transporter2.Set(1);
  } 
  else if(gamepad.GetLeftTriggerAxis() >= .7 && noteCheck.Get() == false){
    pickup.Set(1);
    transporter1.Set(.6);
    transporter2.Set(.3);
  }
  else{
    pickup.Set(0);
    transporter1.Set(0);
    transporter2.Set(0);
  }

  // Shooter Off/On
  if(gamepad.GetLeftBumper()){
    topShooter.Set(0);
    bottomShooter.Set(0);
  }
  else if(gamepad.GetRightBumper()){
    topShooter.Set(-tShotSpeed);
    bottomShooter.Set(bShotSpeed);
  }

  // Shooter Speed
  if(gamepad.GetPOV() == 0){
    tShotSpeed = 1;
    bShotSpeed = 1;
  }
  else if(gamepad.GetPOV() == 180){
    tShotSpeed = .0;
    bShotSpeed = .5;
  }
  else if(gamepad.GetPOV() == 90){
    tShotSpeed = .9;
    bShotSpeed = .4;
  }
  else if(gamepad.GetPOV() == 270){
    tShotSpeed = .875;
    bShotSpeed = .375;
  }
  

  

  // Climber
  if(rightDriveStick.GetTrigger()){
    climber.Set(climber.kReverse);
  }
  else if(leftDriveStick.GetTrigger()){
    climber.Set(climber.kForward);
  }
  else{
    climber.Set(climber.kOff);
  }

}

void Robot::DisabledInit() {
//   if(lclimber.GetFwdChannel() || rclimber.GetFwdChannel()){
//     lclimber.Set(lclimber.kReverse);
//     rclimber.Set(rclimber.kReverse);
//   }
}

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
  double difference = (-1 * rDistance) - (lDistance);
  drive.TankDrive((throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
    if (!sdfr){
    lDriveEncoder.SetPosition(0);
    rDriveEncoder.SetPosition(0);
    sdfr = true;
  }
  drive.TankDrive((0.25 * lDistance),(-0.25 * rDistance), false);
}
int Robot::TurnToAngle(double angle){
  if(!tdr){
  gyro.ZeroYaw();
  tdr = true;
  }
  if(angle < 0){
    if(gyro.GetYaw() >= angle){
      drive.TankDrive(.3,.3,false);
      return NOTDONEYET;
    }
    else{
      drive.TankDrive(0,0,false);
      return DONE;
    }

  }
  else{
    if(gyro.GetYaw() <= angle){
      drive.TankDrive(-.3,-.3,false);
    }
    else{
      drive.TankDrive(0,0,false);
      return DONE;
    }

  }
  return DONE;
}

void Robot::Abort(){
  pickup.StopMotor();
  transporter1.StopMotor();
  transporter2.StopMotor();
  topShooter.StopMotor();
  bottomShooter.StopMotor();
  climber.Set(frc::DoubleSolenoid::Value::kReverse);
  bar4.Set(frc::DoubleSolenoid::Value::kReverse);
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
  static bool FirstCallFlag = true;
  static float autoStartSpeed;
  static double speedUpDistance, slowDownDistance;
  double difference = (-1 * rDistance) - (lDistance);
  if (FirstCallFlag) {
    printf("DD First Call: ");
    if(!sdfr){
      printf("Reset encoders! ");
      lDriveEncoder.SetPosition(0);
      rDriveEncoder.SetPosition(0);
      lDistance = 0;
      rDistance = 0;
      sdfr = true;
      }
      int direction;
      //Changing Direction based off of distance float
      if(speed < 0){
      direction = -1;
      }  
      else{
      direction = 1;
      }

      //adding direction to start speed
      autoStartSpeed = AUTOSTARTSPEED * direction;

      //setting the ramp up and down parameters
      if (distance < (DRIVERAMPUPDISTANCE * 3)) {
        speedUpDistance = distance / 3;
        slowDownDistance = speedUpDistance;
      } else {
        speedUpDistance = DRIVERAMPUPDISTANCE;
        slowDownDistance = distance - DRIVERAMPUPDISTANCE;
      }
      FirstCallFlag = false;
      printf("Direction: %i\n");
   }
  

  
  lDistance = lDriveEncoder.GetPosition() * 1.945;
  rDistance = rDriveEncoder.GetPosition() * 1.945;
if(fabs(lDistance) < fabs(distance)){

    if(lDistance < speedUpDistance){
       drive.TankDrive(-1 * (autoStartSpeed - (difference * 0.1)), (autoStartSpeed + (difference * 0.1)), false);
          std::cout << "start driving" << std::endl;
          return NOTDONEYET;
     }
     else if(lDistance > slowDownDistance){
       drive.TankDrive(-1 * (speed - (difference * 0.1)), (speed + (difference * 0.1)), false);
          std::cout << "end driving" << std::endl;
          return NOTDONEYET;
    }
    else{
      drive.TankDrive(-1 * (autoStartSpeed - (difference * 0.1)), (autoStartSpeed + (difference * 0.1)), false);
         std::cout << "normally driving" << std::endl;
         return NOTDONEYET;
    }
}
  printf("DistanceDrive finished.\n");
  printf("\tLeft Distance: %f\n\tTarget Distance: %f\n",lDistance, distance);
  printf("Drivetrain difference: %f\n", difference);
  drive.TankDrive(0,0,false);
  FirstCallFlag = true;
  return DONE;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
