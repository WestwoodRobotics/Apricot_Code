package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.transport.ElevatorSubsystem;

public class MoveElevator extends CommandBase {

  ElevatorSubsystem elevator;
  double power;

  public MoveElevator(ElevatorSubsystem e, double power) {
    
    this.power = power;
    elevator = e;
    addRequirements(e);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setElevatorPower(power);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
  
}
