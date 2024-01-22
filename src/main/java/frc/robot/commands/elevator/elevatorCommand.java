package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.elevator.*;
import edu.wpi.first.wpilibj.XboxController;


public class elevatorCommand extends CommandBase {

  private Elevator m_Elevator; 
  private XboxController controller;
  

  public elevatorCommand(Elevator elevator, XboxController controller) {
    m_Elevator = elevator;
    this.controller = controller;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //Use controller buttons (A, B, X, Y) to set elevator position
    if (controller.getXButtonPressed()) {
      m_Elevator.setPosition(ElevatorPositions.CLIMB_INIT);
    }
    if (controller.getBButtonPressed()) {
      m_Elevator.setPosition(ElevatorPositions.CLIMB_HOME);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}




