package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.test.*;
import edu.wpi.first.wpilibj.XboxController;


public class testCommand extends CommandBase {

  private Test m_test; 
  private XboxController controller;
  

  public testCommand(Test test, XboxController controller) {
    m_test = test;
    this.controller = controller;
    addRequirements(test);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Use the trigger value for speed
    double triggerValue = MathUtil.clamp(controller.getRightTriggerAxis(), 0, 1);
    double leftTriggerValue = MathUtil.clamp(controller.getLeftTriggerAxis(), 0, 1);
    
    if (triggerValue > 0){
      m_test.setSpeed(triggerValue);
    }
    else if (leftTriggerValue > 0){
      m_test.setSpeed(-leftTriggerValue);
    }
    else{
      m_test.setSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}




