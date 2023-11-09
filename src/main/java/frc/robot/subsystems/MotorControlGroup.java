
    package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MotorControlGroup {
    private final CANSparkMax[] motors;

    // Constructor that takes in an array of CANSparkMax motors
    public MotorControlGroup(CANSparkMax... motors) {
        this.motors = motors;
    }

    // Returns the number of motors in the group
    public double getNumMotors() {
        return motors.length;
    }

    // Sets the power of all motors in the group to the given value
    public void setPower(double power) {
        for (CANSparkMax motor : motors) {
            motor.set(power);
        }
    }


    //Set the defaultBrakeMode of all motors in the control group
    public void setDefaultBrakeMode(boolean brakeMode){
        for (CANSparkMax motor : motors) {
            motor.setIdleMode(brakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
        }
    }



    // Sets the position of all motors in the group to the given value, with the given feedforward value
    public void setPosition(double position, float ff) {
        for (CANSparkMax motor : motors) {
            motor.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
        }
    }

    public void setInverted(boolean inverted){
        for (CANSparkMax motor : motors) {
            motor.setInverted(inverted);
        }
    }

    public void setInverted(boolean inverted, int motorIndex){
        motors[motorIndex].setInverted(inverted);
    }

    
    
    // Returns the power of the first motor in the group (all motors should be the same)
    public double getPower() {
        return motors[0].get();
    }

    // Returns the position of the first motor in the group (all motors should be the same)
    public double getPosition() {
        return motors[0].getEncoder().getPosition();
    }

}
    

