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

    // Apply the non-linear transformation to smooth out the input TODO: Test out the code
    // leftX = Math.copySign(Math.pow(leftX, 2), leftX);
    // leftY = Math.copySign(Math.pow(leftY, 2), leftY);
    // rightX = Math.copySign(Math.pow(rightX, 2), rightX);
    // rightY = Math.copySign(Math.pow(rightY, 2), rightY);

    if (slowMode) {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }

    // Use the trigger value for speed
    double triggerValue = MathUtil.clamp(controller.getRightTriggerAxis(), 0, 1);

    if ((YuMode) && (((rightX != 0) || (rightY != 0) || (leftX != 0)))){
      m_swerveDrive.drive(rightY, rightX, leftX, true, false);
    }
    else if ((leftX != 0) || (leftY != 0) || (rightX != 0)){
      m_swerveDrive.drive(leftY, leftX, rightX, true, false);
      //m_swerveDrive.drive(triggerValue,leftX, leftY, rightX, true, true);
    }
    else{
      m_swerveDrive.setX();
    }


  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}




