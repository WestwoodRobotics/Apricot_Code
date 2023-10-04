// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorModule;

public class ElevatorPosSet extends CommandBase {
  

  private ElevatorModule elevatorModule;
  private String position = new String();

  private double homeTicks = 0;
  private double mid_cone_ticks = 0;
  private double high_cube_ticks = 0;
  private double high_cone_ticks = 0;
  private double cube_pickup_ticks = 0;
  private double cone_pickup_ticks = 0;
  


  public ElevatorPosSet(ElevatorModule elevatorModule, String position) {

    this.elevatorModule = elevatorModule;
    this.position = position;
    addRequirements(elevatorModule);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (position == ("home/low_cube")){
        elevatorModule.setElevatorTicks(homeTicks);
    }
    else if(position == ("cone_mid")){
        elevatorModule.setElevatorTicks(mid_cone_ticks);
    }
    else if(position == ("cube_high")){
        elevatorModule.setElevatorTicks(high_cube_ticks);
    }
    else if (position == ("cone_high/cube_mid")){
        elevatorModule.setElevatorTicks(high_cone_ticks);
    }
    else if (position == ("cube_pickup")){
        elevatorModule.setElevatorTicks(cube_pickup_ticks);
    }
    else if (position == ("cone_pickup")){
        elevatorModule.setElevatorTicks(cone_pickup_ticks);
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
