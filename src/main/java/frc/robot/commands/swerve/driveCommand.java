package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;


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
  public void initialize() {
    slowMode = false;
    YuMode = false; 
  }

  @Override
  public void execute() {
    double leftX, leftY, rightX, rightY;
    if (controller.getBackButtonPressed()) {
      slowMode = !slowMode;
    }
    if (controller.getStartButton()) {
      YuMode= !YuMode;
    }

    leftX = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    leftY = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
    rightX = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);
    rightY = -MathUtil.applyDeadband(controller.getRightY(), OIConstants.kDriveDeadband);

    if (slowMode) {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }


    // Use the trigger value for speed
    double triggerValue = MathUtil.clamp(controller.getRightTriggerAxis(), 0, 1);

    if (YuMode){
      // m_swerveDrive.drive(rightY, rightX, leftX, true, true);
      m_swerveDrive.TriggerDrive(triggerValue,rightX, rightY, leftX, true, true);
    }
    else{
      // m_swerveDrive.drive(leftY, leftX, rightX, true, true);
      m_swerveDrive.TriggerDrive(triggerValue,leftX, leftY, rightX, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}




