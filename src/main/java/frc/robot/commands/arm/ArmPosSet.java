// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmModule;

public class ArmPosSet extends CommandBase {
  
  private ArmModule armModule;
  private String position = new String();

  private double coneTicks = 0;
  private double cubeTicks = 0;

  public ArmPosSet(ArmModule armModule, String position) {
    this.armModule = armModule;
    this.position = position;
    addRequirements(armModule);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (position == ("cone")){
        armModule.setArmTicks(coneTicks);
    }

    else if(position == ("cube")){
        armModule.setArmTicks(cubeTicks);
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
