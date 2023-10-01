// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmModule extends SubsystemBase {
  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;
  
  public ArmModule()
  {
    armMotor = new CANSparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
    armMotor.setInverted(true);
    armEncoder = armMotor.getEncoder();
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
  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
