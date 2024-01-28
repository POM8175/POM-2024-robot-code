package frc.robot.Subsystems.intake_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Subsystems.PomSubsystem;
import static frc.robot.Constants.IntakeConstants.*;


public class IntakeRollerSubsystem extends PomSubsystem
{
    CANSparkMax rollerMotor = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    private final SparkPIDController rollerMotorPidController = rollerMotor.getPIDController();

    // Color Sensor
    //------------------------------------------------------------------------------------
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    ColorMatchResult match;
    final Color noteColor = new Color(130, 98, 26);
    final Color blueColor = new Color(54, 113, 86);
    //------------------------------------------------------------------------------------

    public IntakeRollerSubsystem()
    {
        // setting the PID controllers properties
        double kI = 0,kP = 0,kD = 0,kFF = 0;
        rollerMotorPidController.setP(kP);
        rollerMotorPidController.setI(kI);
        rollerMotorPidController.setD(kD);
        rollerMotorPidController.setFF(kFF);


        m_colorMatcher.addColorMatch(blueColor);
        m_colorMatcher.addColorMatch(noteColor);
    }
    // the subsystems functions
    @Override
    public void stopMotor()
    {
        rollerMotor.set(0);
    }

    @Override
    public void setMotor(double speed)
    {
        rollerMotor.set(speed);
    }

    @Override
    public void resetEncoder()
    {
        rollerMotorEncoder.setPosition(0);
    }

    @Override
    public double getEncoderPosition()
    {
        return rollerMotorEncoder.getPosition();
    }

    @Override
    public void setSetPoint(double target)
    {
        rollerMotorPidController.setReference(target, ControlType.kPosition);
    }

    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 
}
