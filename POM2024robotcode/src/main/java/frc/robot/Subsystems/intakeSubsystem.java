package frc.robot.Subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase
{
    private final ColorMatch colorMatcher = new ColorMatch();
    ColorMatchResult match;
    final Color noteColor = new Color(130, 98, 26);
    final Color blueColor = new Color(54, 113, 86);
    
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public intakeSubsystem()
    {
        colorMatcher.addColorMatch(blueColor);
        colorMatcher.addColorMatch(noteColor);
    }

    public boolean isNoteIn()
    {
        match = colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == blueColor;
    }
}
