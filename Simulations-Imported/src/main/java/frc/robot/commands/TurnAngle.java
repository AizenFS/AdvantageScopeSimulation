// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAngle extends PIDCommand {

  private DriveSubsystem driveSubsystem;
  private double setPoint;
  
  public TurnAngle(DriveSubsystem driveSubsystem, double setPoint) {
    super(
        new PIDController(Constants.DriveConstants.kTurnP, Constants.DriveConstants.kTurnI,Constants.DriveConstants.kTurnD),
        driveSubsystem::getHeading,
        setPoint,
        output -> driveSubsystem.arcadeDrive(0, output),
        driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.setPoint = setPoint;
    getController().enableContinuousInput(-180, 180);
    getController()
    .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    
  }

  public void initialize(){
    //driveSubsystem.resetGyro();
  }
  
  @Override
  public boolean isFinished() {
    if(driveSubsystem.getHeading()>setPoint*0.9){
      return true;
    }
    return false;
  }
}