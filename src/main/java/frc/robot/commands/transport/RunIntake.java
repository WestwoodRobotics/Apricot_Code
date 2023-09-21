package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.transport.IntakeSubsystem;

public class RunIntake extends CommandBase {

  IntakeSubsystem intake;
  double power;

  public RunIntake(IntakeSubsystem e, double power) {
    
    this.power = power;
    intake = e;
    addRequirements(e);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setIntakePower(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
  
}
