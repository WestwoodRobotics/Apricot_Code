package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorModule extends SubsystemBase{
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final PIDController elevPidController;


    public ElevatorModule()
    {
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor, MotorType.kBrushless);
        elevatorMotor.setInverted(false);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevPidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        elevPidController.setTolerance(1);
        resetEncoder();
    }
    public boolean atSetpoint(){
        return elevPidController.atSetpoint();
    }

    public void setElevatorPower(double power)
    {
        elevatorMotor.set(power);
    }

    public double getEncoderPosition()
    {
        return elevatorEncoder.getPosition();
    }

    public void resetEncoder() {
        elevatorEncoder.setPosition(0);
    }

    public void setElevatorTicks(double ticks) {

        elevatorMotor.setVoltage(elevPidController.calculate(elevatorEncoder.getPosition(), ticks));
       while(ticks > this.getEncoderPosition()){
            elevatorMotor.set(0.5);
            System.out.println("AT ELEVATOR1 MODULE!");
            break;
       } 
       while (ticks < this.getEncoderPosition()){
            elevatorMotor.set(-0.5);
            System.out.println("AT ELEVATOR2 MODULE!");
            break;
       } 
       elevatorMotor.set(0);
    }
    

}
