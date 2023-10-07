// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.elevator.ElevatorModule;


public class ElevatorCommand extends CommandBase {
  
  private XboxController controller;
  private XboxController controller2;
  private ElevatorModule elevatorModule;

  private POVButton dPadUp;
  private POVButton dPadDown;

  public ElevatorCommand(ElevatorModule elevatorModule, XboxController controller, XboxController controller2) {
    this.controller = controller;
    this.controller2 = controller2;
    this.elevatorModule = elevatorModule;
    dPadDown = new POVButton(controller2, 0);
    dPadUp = new POVButton(controller2,180);
  

    addRequirements(elevatorModule);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((controller2.getRawAxis(1) > 0.1) || (controller2.getRawAxis(1) < -0.1)) {//TODO: Make sure Axis Number is Correct
      elevatorModule.setElevatorPower(controller2.getRawAxis(1));
    }
    else{
      elevatorModule.setElevatorPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}