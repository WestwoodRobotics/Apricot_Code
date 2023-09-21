package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TransportConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor;


  // Constructor for initializing the intake module
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(PortConstants.kIntakeMotorPort, MotorType.kBrushless);
    intakeMotor.setInverted(false);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public void setIntakePosition(double position, float ff) {
    intakeMotor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
  }

  public double getIntakePosition(){
    return intakeMotor.getEncoder().getPosition();
  }

}