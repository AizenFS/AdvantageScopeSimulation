// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveDis;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  XboxController controller = new XboxController(0);

 
  public RobotContainer() {
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand( () ->
     driveSubsystem.arcadeDrive(-controller.getLeftY(), controller.getRightX()),driveSubsystem));
  }

  private void configureButtonBindings() {
    new JoystickButton(controller, Button.kA.value)
        .onTrue(new InstantCommand(
        () -> driveSubsystem.resetOdometry(new Pose2d())));
        
  }

  public Command getAutonomousCommand() {
    driveSubsystem.resetOdometry(new Pose2d(1,2,new Rotation2d()));
      return new DriveDis(driveSubsystem,5);
      
       
  }               
  public DriveSubsystem getRobotDrive() {
    return driveSubsystem;
  }
  public void zeroAllOutputs() {
    driveSubsystem.tankDriveVolts(0, 0);
  }
}
