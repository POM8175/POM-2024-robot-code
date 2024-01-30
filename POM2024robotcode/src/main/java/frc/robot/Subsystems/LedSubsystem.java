package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LedsConstants.*;

public class LedSubsystem extends SubsystemBase {
    
    AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    Color c = new Color(2,2,2);
    
    public LedSubsystem(){
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(NUM_LEDS);
        m_led.start();
        setLeds(POM_PURPLE);
    }

    @Override
    public void periodic(){
        // tab.add("Red", 0).getEntry();
        // tab.add("Green", 0).getEntry();
        // tab.add("Blue", 0).getEntry();
    }

    public void setLeds(Color color){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setLED(i, color);
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

    public void ShuffleboardColor(){
       
    }
}

