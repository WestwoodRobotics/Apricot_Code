// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmModule;

public class ArmPosSet extends CommandBase {
  
  private ArmModule armModule;
  private String position = new String();

  private double cone_pickup_ticks = 100;
  private double cube_pickup_ticks = 70;
  private double homeTicks = 0;
  private double mid_cone_ticks = 0;
  private double high_cube_ticks = 0;
  private double high_cone_ticks = 0;

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

    if (position == ("home/low_cube")){
      armModule.setArmTicks(homeTicks);
  }
  else if(position == ("cone_mid")){
      armModule.setArmTicks(mid_cone_ticks);
  }
  else if(position == ("cube_high")){
      armModule.setArmTicks(high_cube_ticks);
  }
  else if (position == ("cone_high/cube_mid")){
      armModule.setArmTicks(high_cone_ticks);
  }
  else if (position == ("cube_pickup")){
      armModule.setArmTicks(cube_pickup_ticks);
  }
  else if (position == ("cone_pickup")){
      armModule.setArmTicks(cone_pickup_ticks);
  }
  else{
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
