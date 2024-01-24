package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDis extends PIDCommand {

  private DriveSubsystem driveSubsystem;
  private double setPoint;
  
  public DriveDis(DriveSubsystem driveSubsystem, double setPoint) {
    super(
        new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
        driveSubsystem::getBothEncoders,
        setPoint,
        output -> driveSubsystem.arcadeDrive(output,0),
        driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.setPoint = setPoint;

    getController()
        .setTolerance(DriveConstants.kDriveToToleranceMeters, DriveConstants.kDriveRateToleranceMetersPerS);
    getController().reset();
    
  }

  @Override
  public void initialize(){
    driveSubsystem.resetEncoders();
  }

  @Override
  public boolean isFinished() {    
     if(driveSubsystem.getBothEncoders()> setPoint*0.95){
      return true;
     }
     return false;
    
  }
}