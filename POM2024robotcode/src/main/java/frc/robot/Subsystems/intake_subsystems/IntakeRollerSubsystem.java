package frc.robot.Subsystems.intake_subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.PomSubsystem;

import static frc.robot.Constants.IntakeConstants.*;


public class IntakeRollerSubsystem extends PomSubsystem
{
    // Motors
    //------------------------------------------------------------------------------------
    VictorSPX rollerMotor = new VictorSPX(ROLLER_MOTOR);    
    VictorSPX rollerMotorSlave = new VictorSPX(ROLLER_MOTOR_SLAVE);    
    //------------------------------------------------------------------------------------
    
    // the power given to the motor
    double motorSet = 0;
    
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
        // adding collors to the dataset of m_colorMatcher
        m_colorMatcher.addColorMatch(blueColor);
        m_colorMatcher.addColorMatch(noteColor);

        // setting rollerMotorSlave to follow rollerMotor
        rollerMotorSlave.follow(rollerMotor);
    }
    // the subsystems functions

    // stoping the motor
    @Override
    public void stopMotor()
    {
        rollerMotor.set(VictorSPXControlMode.PercentOutput,0);
    }

    // setting the motors speed
    @Override
    public void setMotor(double speed)
    {
        rollerMotor.set(VictorSPXControlMode.PercentOutput,speed);
    }

    // a boolean function that checks if the color sensor sees a note
    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 

    // the subsystems commands
    public Command intakeNoteCommand = Commands.startEnd(() -> setMotor(ROLLER_MOTOR_SPEED),() -> stopMotor(), this);
    public Command outakeNoteCommand = Commands.startEnd(() -> setMotor(-ROLLER_MOTOR_SPEED), () -> stopMotor(), this);
}
