package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.LED;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class LEDCommand extends CommandBase {
    private final LED m_led;
    private final DigitalInput limitSwitch;

    public LEDCommand(LED led, int limitSwitchPort) {
        m_led = led;
        limitSwitch = new DigitalInput(limitSwitchPort);
        addRequirements(m_led);  // This command requires the LED subsystem
    }

    @Override
    public void execute() {
        // Check the state of the limit switch and set the color of the LEDs accordingly
        if (limitSwitch.get()) {
            // If the limit switch is pressed, set the LEDs to green
            m_led.setColor(0, 255, 0);
        } else {
            // If the limit switch isn't pressed, set the LEDs to orange
            m_led.setColor(255, 165, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off the LEDs when the command is finished
        m_led.setColor(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately, but you can change this to suit your needs
        return false;
    }
}