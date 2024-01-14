package frc.robot.subsystems.test;

import com.revrobotics.CANSparkMax;




import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import constants.java
import frc.robot.Constants.*;
import frc.robot.subsystems.MotorControlGroup;




public class Test extends SubsystemBase {
    private CANSparkMax motor1;
    private MotorControlGroup motor;

    public Test() {
        motor1 = new CANSparkMax(ModuleConstants.testCANId, CANSparkMax.MotorType.kBrushless); // Replace 0 with the appropriate motor port number
        motor = new MotorControlGroup(motor1);
    }

    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

    public double getSpeed() {
        return motor.getPower();
    }
}