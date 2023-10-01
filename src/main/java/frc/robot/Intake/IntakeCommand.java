package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeModule;

public class IntakeCommand extends CommandBase {
    // useful variables
    private XboxController controller;
    private IntakeModule intakeModule;

    public IntakeCommand(IntakeModule intakeModule, XboxController controller) {
        this.controller = controller;
        this.intakeModule = intakeModule;
        addRequirements(intakeModule);

    
    }

    @Override
    public void execute()
    {
        if (controller.getLeftTriggerAxis() > 0) {
            intakeModule.setIntakePower(1);
        } else if (controller.getRightTriggerAxis() > 0) {
            intakeModule.setIntakePower(-1);
        }
        else if (controller.getLeftTriggerAxis() == 0 && controller.getRightTriggerAxis() == 0)
        {
            intakeModule.setIntakePower(0);
        }
        //change to whileHeld command (OnTrue?)
    }
}
