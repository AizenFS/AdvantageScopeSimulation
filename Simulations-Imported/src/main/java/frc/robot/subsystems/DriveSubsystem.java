// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.io.Console;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax m_left = new PWMSparkMax(1);
  private final PWMSparkMax leftMotor2 = new PWMSparkMax(2);
  private final PWMSparkMax m_right = new PWMSparkMax(3);
  private final PWMSparkMax rightMotor2 = new PWMSparkMax(4);

  private final DifferentialDrive m_robotDrive = 
    new DifferentialDrive(m_left/* ::set*/ ,m_right/* ::set*/);

  private final DifferentialDriveOdometry odometry;
 
  private Encoder leftEncoder = 
    new Encoder(0, 1,false);
  private Encoder rightEncoder = 
    new Encoder(2, 3,true);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  public DifferentialDrivetrainSim drivetrainSim;

  private EncoderSim leftEncoderSim;  
  private EncoderSim rightEncoderSim;

  private final Field2d field2d;

  private final double start_angel = 0;
  
  public DriveSubsystem() {
    resetEncoders();

    SendableRegistry.addChild(m_robotDrive, m_left);
    SendableRegistry.addChild(m_robotDrive, m_right);

    m_left.addFollower(leftMotor2);
    m_right.addFollower(rightMotor2);

    m_right.setInverted(true);

    leftEncoder.setDistancePerPulse
      (DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse
      (DriveConstants.kEncoderDistancePerPulse);

    odometry = new DifferentialDriveOdometry
      (Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),rightEncoder.getDistance());

    if (RobotBase.isSimulation()) { 
      drivetrainSim = DifferentialDrivetrainSim.createKitbotSim
        (KitbotMotor.kDoubleNEOPerSide, 
        KitbotGearing.k12p75,
        KitbotWheelSize.kSixInch,
        null);   

      field2d = new Field2d();      
      SmartDashboard.putData("Field", field2d);

      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
    } 
    else {
      field2d = null;
      leftEncoderSim = null;
      rightEncoderSim = null;
    }

  }
  @Override
  public void periodic() {
    odometry.update(      
      Rotation2d.fromDegrees(
      getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance());

    field2d.setRobotPose(getPose());
    System.out.println("hallo");
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs
    (m_left.get()*RobotController.getBatteryVoltage(),
    m_right.get()*RobotController.getBatteryVoltage());

    drivetrainSim.update(0.02);

    leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());

    rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());

    int dev = 
      SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = 
      new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,"Yaw"));
    angle.set(-drivetrainSim.getHeading().getDegrees());

    Logger.recordOutput("gyro",angle.get());

  }   

  public void setMotors(double leftSpeed, double rightSpeed) {
    m_left.set(leftSpeed);
    m_right.set(rightSpeed);
  }
  
  public void resetGyro(){
    gyro.reset();
  } 
  
  public void arcadeDrive(double str, double turn){
    m_robotDrive.arcadeDrive(str, turn);
  }

  public void tankDrive(double left, double right){
    m_robotDrive.tankDrive(left, right);
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public boolean isYawChanged(){
    if(Math.abs(getYaw()) > 15){   
      return true;
    }
    return false;
  }

  @AutoLogOutput
  public double getYaw(){
    double angle = gyro.getYaw() + start_angel;
    if(angle > 180){
      angle -= 360;
    }
    return angle;
  }

  @AutoLogOutput
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * 
      (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @AutoLogOutput
  public double getRightEncoder(){
    return rightEncoder.getDistance();
  }

  @AutoLogOutput
  public double getleftEncoder(){
    return leftEncoder.getDistance();
  }

  @AutoLogOutput
  public double getBothEncoders(){   
    return (getRightEncoder() + getleftEncoder())/2;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {    
    resetEncoders();
    drivetrainSim.setPose(pose);
    odometry.resetPosition
      (Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance(),
      pose);
  }

  public double getDrawnCurrentAmps() {
    return drivetrainSim.getCurrentDrawAmps();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    m_robotDrive.feed();
  }

}
