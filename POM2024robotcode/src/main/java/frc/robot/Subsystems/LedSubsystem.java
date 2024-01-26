package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LedsConstants.*;

public class LedSubsystem extends SubsystemBase {
    
    AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;

    
    public LedSubsystem(){
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(180);
        m_led.setLength(180);
        m_led.start();
        setLeds(148,0,211);
    }

    public void setLeds(int red, int green, int blue){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
    }

    public void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }
}

