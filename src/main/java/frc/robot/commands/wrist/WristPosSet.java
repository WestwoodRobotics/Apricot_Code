// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.WristModule;

public class WristPosSet extends CommandBase {
  
  private WristModule armModule;
  private String position = new String();
  private int positionNum;


  public WristPosSet(WristModule armModule, String position) {
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
    System.out.print("Executed");

    if (position == ("home/low_cube")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cube_outtake);
      positionNum = 0;
  }
  else if(position == ("cone_mid")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cone_pickup);
      positionNum = 1;
  }
  else if(position == ("cube_high")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cube_outtake);
      positionNum = 2;
  }
  else if (position == ("cone_high/cube_mid")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cube_pickup);
      positionNum = 3;
  }
  else if (position == ("cube_pickup")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cube_pickup);
      positionNum = 4;
  }
  else if (position == ("cone_pickup")){
      armModule.setArmTicks(Constants.ArmConstants.arm_cone_pickup);
      positionNum = 5;
  }
  else{
      armModule.setArmPower(0);
      positionNum = 0;
  }

  }

  public void wristSetPos(String position)
  {
    this.position = position;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armModule.atSetpoint()){
      return true;
    }
    return false;
  }
}
