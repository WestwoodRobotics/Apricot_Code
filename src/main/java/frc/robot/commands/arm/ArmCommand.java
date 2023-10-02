// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmModule;

public class ArmCommand extends CommandBase {
  private XboxController controller;
  private ArmModule armModule;
  
  public ArmCommand(ArmModule armModule, XboxController controller) {
    this.controller = controller;
    this.armModule = armModule;
    addRequirements(armModule);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getXButton()) {
      armModule.setArmPower(0.75);
    } else if (controller.getYButton())
    {
      armModule.setArmPower(-0.75);
    } else if (controller.getXButton() == false && controller.getYButton() == false)
    {
      armModule.setArmPower(0);
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
