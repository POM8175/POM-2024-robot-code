package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends PomSubsystem
{
    VictorSPX transferMotor = new VictorSPX(0);
    
    // Color Sensor
    //------------------------------------------------------------------------------------
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    ColorMatchResult match;
    final Color noteColor = new Color(130, 98, 26);
    final Color blueColor = new Color(54, 113, 86);
    //------------------------------------------------------------------------------------

    public TransferSubsystem()
    {
        // adding collors to the dataset of m_colorMatcher
        m_colorMatcher.addColorMatch(blueColor);
        m_colorMatcher.addColorMatch(noteColor);
    }

    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 

    @Override
    public void setMotor(double speed)
    {
        transferMotor.set(ControlMode.PercentOutput,speed);
    }

    @Override
    public void stopMotor()
    {
        transferMotor.set(ControlMode.PercentOutput,0);
    }

    public Command getNote()
    {
        return new StartEndCommand(() -> setMotor(1), () -> stopMotor(), this).until(() -> isNoteIn());
    }
    public Command transferNote()
    {
        return new StartEndCommand(() -> setMotor(1), () -> stopMotor(), this).until(() -> !isNoteIn());
    }
}
