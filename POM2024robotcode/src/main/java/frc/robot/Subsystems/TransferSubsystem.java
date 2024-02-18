package frc.robot.Subsystems;

import static frc.robot.Constants.TransferConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TransferSubsystem extends PomSubsystem
{
    CANSparkMax transferMotor = new CANSparkMax(TRANSFER_MOTOR, MotorType.kBrushless);
    // Color Sensor
    //------------------------------------------------------------------------------------
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    ColorMatchResult match;
    //------------------------------------------------------------------------------------

    public TransferSubsystem()
    {
        // adding collors to the dataset of m_colorMatcher
        for(int i = 0;i<notNoteColors.length;i++) m_colorMatcher.addColorMatch(notNoteColors[i]);
        
        m_colorMatcher.addColorMatch(noteColor);
        transferMotor.setIdleMode(IdleMode.kBrake);
        setDefaultCommand(this.runOnce(() -> stopMotor()));
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("transfer current speed", transferMotor.get());
    }

    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 

    @Override
    public void setMotor(double speed)
    {
        transferMotor.set(speed);
        transferMotor.set(speed);
    }

    @Override
    public void stopMotor()
    {
        transferMotor.set(0);
        transferMotor.set(0);
    }

    // the subsystems commands
    public Command getFromIntake()
    {
        return this.startEnd(() -> setMotor(-0.25), () -> stopMotor()).until(() -> isNoteIn());
    }
    public Command transfer(boolean isToShooter)
    {
        return this.startEnd(() -> setMotor(isToShooter ? TRANSFER_SPEED : -TRANSFER_SPEED), () -> setMotor(isToShooter ? TRANSFER_SPEED : -TRANSFER_SPEED)).until(() -> !isNoteIn()).andThen(new WaitCommand(TRANSFER_TIME_OUT)).andThen(() -> stopMotor(), this);
    }

    public Command joystickShootCommand(DoubleSupplier sup)
    {
        return run(() -> setMotor(sup.getAsDouble()));
    }
}
