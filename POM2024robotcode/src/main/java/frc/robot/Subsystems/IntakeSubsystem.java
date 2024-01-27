package frc.robot.Subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class IntakeSubsystem extends SubsystemBase 
{
    // Color Sensor
    //------------------------------------------------------------------------------------
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    ColorMatchResult match;
    final Color noteColor = new Color(130, 98, 26);
    final Color blueColor = new Color(54, 113, 86);
    //------------------------------------------------------------------------------------
    //the subsystems composing this subsystem

    public IntakeRollerSubsystem rollerSubsystem = new IntakeRollerSubsystem();


    public IntakeSubsystem()
    {
        m_colorMatcher.addColorMatch(blueColor);
        m_colorMatcher.addColorMatch(noteColor);
    }

    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 
}
