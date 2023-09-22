package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.MotorControlGroup;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax elevatorMotor1;
  private final CANSparkMax elevatorMotor2;
  private final MotorControlGroup elevatorMotors;


  // Constructor for initializing the Elevator Subsystem
  public ElevatorSubsystem() {
    elevatorMotor1 = new CANSparkMax(PortConstants.kElevatorMotor1Port, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(PortConstants.kElevatorMotor2Port, MotorType.kBrushless);

    elevatorMotors = new MotorControlGroup(elevatorMotor1, elevatorMotor2);

    elevatorMotors.setInverted(false);
  }

  public void setElevatorPower(double power) {
    elevatorMotors.setPower(power);
  }

  public void setElevatorPosition(double position, float ff) {
    elevatorMotors.setPosition(position, ff);
  }

  public double getElevatorPosition(){
    return elevatorMotors.getPosition();
  }

}