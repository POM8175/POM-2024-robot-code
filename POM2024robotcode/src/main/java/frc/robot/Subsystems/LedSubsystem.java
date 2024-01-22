package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LedsConstants.*;

public class LedSubsystem extends SubsystemBase {
    
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    
    public LedSubsystem(){
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(NUM_LEDS);
        m_led.start();
        setLeds(148,0,211);
    }

    public void setLeds(int red, int green, int blue){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
    }

}