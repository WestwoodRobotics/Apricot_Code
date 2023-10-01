package frc.robot.subsystems;
/* 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swerve.DriveConstantControlCommand;
//import frc.robot.constant.TransportConstants;
*/

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeModule extends SubsystemBase {
    // Motor for the intake mechanism
    private final CANSparkMax intakeMotor;
    private int intakeMode = 0;
    // private int inverted = 1;

    // Constructor for initializing the intake module
    public IntakeModule() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        intakeMotor.setInverted(false);
    }

    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public double intakeInverted(int mode) {
        if (mode == 1) {
            return -0.75;
        } else {
            return 1;
        }
    }

    public int getIntakeMode() {
        return intakeMode % 2;
    }

    public boolean getCubeMode() {
        if (this.getIntakeMode() == 1)
            return true;
        return false;
    }

    public void incrementMode() {
        intakeMode++;
        SmartDashboard.putNumber("Intake Mode", this.getIntakeMode());
    }
/* we don't use this, so i'll leave it here for later
    public float getRotValue() {
        if (getIntakeMode() == 0) {
            return TransportConstants.WRIST_START_ROT;
        } else {
            return TransportConstants.WRIST_FLIPPED_ROT;
        }
    }
*/
}
