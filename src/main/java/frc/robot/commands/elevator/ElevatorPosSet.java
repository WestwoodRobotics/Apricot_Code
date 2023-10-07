// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorModule;

public class ElevatorPosSet extends CommandBase {
  

  private ElevatorModule elevatorModule;
  private String position = new String();

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
    
    if (position == "home/low_cube"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_cube_pickup);
        
    }
    else if(position == "cone_mid"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_outtake);
    }
    else if(position == "cube_high"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_outtake);
    }
    else if (position == "cone_high/cube_mid"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_outtake);
    }
    else if (position == "cube_pickup"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_outtake);
    }
    else if (position == "cone_pickup"){
        elevatorModule.setElevatorTicks(Constants.ElevatorConstants.elev_outtake);
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
    if(elevatorModule.atSetpoint()){
      return true;
    }
    return false;
  }
}