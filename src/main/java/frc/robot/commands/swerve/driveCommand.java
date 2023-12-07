package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
// import frc.robot.subsystems.swerve.DriveSpeed;

// import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
// import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

public class driveCommand extends CommandBase {

  private final DriveSubsystem m_swerveDrive;


  private XboxController controller;
  private boolean slowMode;
  private boolean YuMode;



  public driveCommand(DriveSubsystem swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;
    addRequirements(swerveDrive);
  }


  @Override
  public void initialize() 
  {
    slowMode = false;
    YuMode = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX, leftY, rightX, rightY;
    if (controller.getBackButtonPressed())
    {
      slowMode = !slowMode;
    }
    if (controller.getStartButton())
    {
      YuMode= !YuMode;
    }



    leftX = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    leftY = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
    rightX = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);
    rightY = -MathUtil.applyDeadband(controller.getRightY(), OIConstants.kDriveDeadband);




    if (slowMode)
    {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }
    if (YuMode){
      m_swerveDrive.drive(rightY, rightX, leftX, true, true);
    }
    else{
      m_swerveDrive.drive(leftY, leftX, rightX, true, true);
    }

  }


  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should never end in teleop
    return false;
  }
}