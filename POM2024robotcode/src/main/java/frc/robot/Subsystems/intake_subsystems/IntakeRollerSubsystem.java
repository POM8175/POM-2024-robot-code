package frc.robot.Subsystems.intake_subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.PomSubsystem;


public class IntakeRollerSubsystem extends PomSubsystem
{
    // Motors
    //------------------------------------------------------------------------------------
    VictorSPX rollerMotor = new VictorSPX(10);    
    VictorSPX rollerMotorSlave = new VictorSPX(11);    
    PIDController rollerPidController;
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

        // setting the values of the PID Controller
        double kI = 0,KD = 0,kP = 0,kF = 0;
        rollerPidController = new PIDController(kP, kI, KD);
    }
    // the subsystems functions

    // stoping the motor
    @Override
    public void stopMotor()
    {
        rollerPidController.setSetpoint(0);
        motorSet = rollerPidController.calculate(motorSet);
        rollerMotor.set(VictorSPXControlMode.PercentOutput,motorSet);
    }

    // setting the motors speed
    @Override
    public void setMotor(double speed)
    {
        rollerPidController.setSetpoint(speed);
        motorSet = rollerPidController.calculate(motorSet);
        rollerMotor.set(VictorSPXControlMode.PercentOutput,motorSet);
    }

    // a boolean function that checks if the color sensor sees a note
    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 
}
