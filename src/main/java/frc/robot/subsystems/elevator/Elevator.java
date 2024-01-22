package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import constants.java
import frc.robot.Constants.*;
import frc.robot.subsystems.MotorControlGroup;
import frc.utils.Position_Enums.ElevatorPositions;




public class Elevator extends SubsystemBase {
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private MotorControlGroup motor;

    public Elevator() {
        motor1 = new CANSparkMax(PortConstants.kElevatorMotor1Port, CANSparkMax.MotorType.kBrushless); 
        motor2 = new CANSparkMax(PortConstants.kElevatorMotor2Port, CANSparkMax.MotorType.kBrushless);
        motor = new MotorControlGroup(motor1, motor2);
    }

    public void setPosition(ElevatorPositions climbInit) {
        if (climbInit == ElevatorPositions.CLIMB_INIT) {
            motor.setPosition(ElevatorConstants.elevatorClimbInit, ElevatorConstants.ff);
        } else if (climbInit == ElevatorPositions.CLIMB_HOME) {
            motor.setPosition(ElevatorConstants.elevatorClimbHome, ElevatorConstants.ff);
        }
    }

    public double getPosition() {
        return motor.getPosition();
    }

    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

    public double getSpeed() {
        return motor.getPower();
    }
}