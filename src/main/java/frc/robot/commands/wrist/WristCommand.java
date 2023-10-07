// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristModule;

public class WristCommand extends CommandBase {
  private XboxController controller;
  private XboxController controller2;
  private WristModule armModule;
  
  public WristCommand(WristModule armModule, XboxController controller, XboxController controller2) {
    this.controller = controller;
    this.controller2 = controller2;
    this.armModule = armModule;
    addRequirements(armModule);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if ((controller2.getRawAxis(0) > 0.1) || (controller2.getRawAxis(0) < 0.1))  { //TODO: Make sure Axus Number is Correct
    }
    else{0
      armModule.setArmPower(0);*/
    armModule.setArmPower(controller2.getRawAxis(5));

  }

  public void setArmTicks() {

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
