// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class WristModule extends SubsystemBase {
  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;
  private final PIDController wristPidController;
  private double currentSetPoint = 0;
  
  public WristModule()
  {
    armMotor = new CANSparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
    armMotor.setInverted(true);
    armEncoder = armMotor.getEncoder();
    wristPidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    wristPidController.setTolerance(1);
    resetEncoder();
  }

  public void setArmPower(double power)
  {
      armMotor.set(power);
  }

  public double getEncoderPosition()
  {
      return armEncoder.getPosition();
  }

  public void resetEncoder() {
      armEncoder.setPosition(0);
  }



    public void setArmTicks(double ticks) {
      currentSetPoint = ticks;
      armMotor.setVoltage(wristPidController.calculate(armEncoder.getPosition(), ticks));
     while(ticks > this.getEncoderPosition()){
          armMotor.set(0.25);
          System.out.println("AT WRIST1 MODULE!");
          break;
     } 
     while (ticks < this.getEncoderPosition()){
          armMotor.set(-0.25);
          System.out.println("AT WRIST2 MODULE!");
          break;
     } 
     armMotor.set(0);
  }

  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
  
  public boolean atSetpoint(){
    return wristPidController.atSetpoint();
  }
  
}