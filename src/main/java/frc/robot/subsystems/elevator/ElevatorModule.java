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
    private double currentSetPoint = 0;

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
        currentSetPoint = ticks;
        elevatorMotor.setVoltage(elevPidController.calculate(elevatorEncoder.getPosition(), ticks));
       while(ticks > this.getEncoderPosition()){
            elevatorMotor.set(0.25);
       } 
       while (ticks < this.getEncoderPosition()){
            elevatorMotor.set(-0.25);
       } 
       elevatorMotor.set(0);
    }
    

}
