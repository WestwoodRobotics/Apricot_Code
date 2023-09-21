package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TransportConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax elevatorMotor;


  // Constructor for initializing the intake module
  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(PortConstants.kElevatorMotorPort, MotorType.kBrushless);
    elevatorMotor.setInverted(false);
  }

  public void setElevatorPower(double power) {
    elevatorMotor.set(power);
  }

  public void setElevatorPosition(double position, float ff) {
    elevatorMotor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
  }

  public double getElevatorPosition(){
    return elevatorMotor.getEncoder().getPosition();
  }

}