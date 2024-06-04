package frc.robot.Subsystems;

import static frc.robot.Constants.TransferConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TransferSubsystem extends PomSubsystem
{
    CANSparkMax transferMotor = new CANSparkMax(TRANSFER_MOTOR, MotorType.kBrushless);
    DigitalInput IrSensor = new DigitalInput(IR_SENSOR_ID);

    public TransferSubsystem()
    {
        transferMotor.setIdleMode(IdleMode.kBrake);
        setDefaultCommand(this.runOnce(() -> stopMotor()));
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("transfer current power", transferMotor.get());
        SmartDashboard.putNumber("transfer current speed", transferMotor.getEncoder().getVelocity());
    }

    public boolean isNoteIn()
    {
        return !IrSensor.get();
    } 

    @Override
    public void setMotor(double speed)
    {
        transferMotor.set(speed);
    }

    @Override
    public void stopMotor()
    {
        transferMotor.set(0);
    }

    // the subsystems commands
    public Command getFromIntake()
    {
        return this.startEnd(() -> setMotor(TRANSFER_INTAKE_SPEED), () -> stopMotor()).until(() -> isNoteIn());
    }
    public Command transfer(boolean isToShooter)
    {
        return this.startEnd(() -> setMotor(isToShooter ? TRANSFER_SHOOT_SPEED : -TRANSFER_SHOOT_SPEED), () -> setMotor(isToShooter ? TRANSFER_SHOOT_SPEED : -TRANSFER_SHOOT_SPEED)).until(() -> !isNoteIn()).andThen(new WaitCommand(TRANSFER_TIME_OUT)).andThen(this::stopMotor, this);
    }

    public Command amp()
    {
         return this.startEnd(() -> setMotor(-0.2), () -> {}).until(() -> !isNoteIn()).andThen(new WaitCommand(TRANSFER_TIME_OUT)).andThen(() -> stopMotor(), this);
    }
    public Command outForShootCommand()
    {
         return this.startEnd(() -> setMotor(0.07), () -> {}).until(() -> !isNoteIn()).andThen(() -> stopMotor(), this);
    }
    public Command inForShootCommand()
    {
         return this.startEnd(() -> setMotor(TRANSFER_SHOOT_SPEED), () -> {}).until(() -> isNoteIn());
    }
    public Command joystickShootCommand(DoubleSupplier sup)
    {
        return run(() -> setMotor(sup.getAsDouble()));
    }

    public Command stopWheelsCommand()
    {
        return this.runOnce(() -> stopMotor());
    }
}
