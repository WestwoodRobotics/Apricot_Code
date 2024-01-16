package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public LED(int pwmPort, int numLEDs) {
        m_led = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(numLEDs);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setColor(int red, int green, int blue) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
    }
}