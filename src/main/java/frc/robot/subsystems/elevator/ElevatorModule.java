package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorModule extends SubsystemBase{
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;

    public ElevatorModule()
    {
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotor, MotorType.kBrushless);
        elevatorMotor.setInverted(false);
        elevatorEncoder = elevatorMotor.getEncoder();
        resetEncoder();
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
       if(ticks > this.getEncoderPosition()){
            elevatorMotor.set(0.25);
       } 
       else if (ticks < this.getEncoderPosition()){
            elevatorMotor.set(-0.25);
       } 
       else {
            elevatorMotor.set(0);
       }
    }

}
