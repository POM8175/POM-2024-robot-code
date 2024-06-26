package frc.robot.Subsystems.led_subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LedsConstants.*;

public class LedSubsystem extends SubsystemBase {
    
    AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    public LedSubsystem(){
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(NUM_LEDS);
        m_led.start();
        setLeds(POM_PURPLE);
        setDefaultCommand(this.runOnce(() -> setLeds(Color.kBlue)));
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

    public void sethalfLeds(Color color1, Color color2){
        for(int i = 0; i < m_ledBuffer.getLength()/2; i++){
            m_ledBuffer.setLED(i,color1);
            m_ledBuffer.setLED(i + (m_ledBuffer.getLength() / 2),color2);
            
        }
            m_led.setData(m_ledBuffer);    
        
    }

    public void setThreeLeds(Color color1, Color color2, Color color3){
        for(int i =0; i < m_ledBuffer.getLength()/3; i++){
            m_ledBuffer.setLED(i, color1);//0
            m_ledBuffer.setLED(i + (m_ledBuffer.getLength() / 3), color2);//60
            m_ledBuffer.setLED(i + (2*(m_ledBuffer.getLength() / 3)), color3);//120
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void setThirdLed(Color color, int third){
        
        for (int i = 60*third - 1; i>(m_ledBuffer.getLength() - m_ledBuffer.getLength()/3 *third); i--){
            m_ledBuffer.setLED(i, color);
        } 
        m_led.setData(m_ledBuffer);
    }


    public void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * NUM_LEDS / m_ledBuffer.getLength())) % NUM_LEDS;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= NUM_LEDS;
        m_led.setData(m_ledBuffer);
    }



    public Command setLedCommand(Color color)
    {
        return this.runOnce(() -> setLeds(color));
    }

    public Command TwoLedCommand(Color color1, Color color2){
        return this.runOnce(() -> {
            sethalfLeds(color1, color2);
            
        });
    }

    public Command ThreeLedCommand(Color color1, Color color2, Color color3){
        return this.runOnce(() ->{
            setThreeLeds(color1, color2, color3);
        });
    }

    public Command thirdLeCommand(Color color, int third){
        return this.runOnce((()-> {
            setThirdLed(color, third);
        }));
    }
    
}

