// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorModule;

public class ElevatorCommand extends CommandBase {
  
  private XboxController controller;
  private ElevatorModule elevatorModule;

  public ElevatorCommand(ElevatorModule elevatorModule, XboxController controller) {
    this.controller = controller;
    this.elevatorModule = elevatorModule;
    addRequirements(elevatorModule);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getAButton()) {
      elevatorModule.setElevatorPower(0.25);
    } else if (controller.getBButton())
    {
      elevatorModule.setElevatorPower(-0.25);
    } else if (controller.getAButton() == false && controller.getBButton() == false)
    {
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
