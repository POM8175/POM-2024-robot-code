package frc.robot.Subsystems.intake_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.PomSubsystem;


public class IntakeRollerSubsystem extends PomSubsystem
{
    CANSparkMax rollerMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    private final SparkPIDController rollerMotorPidController = rollerMotor.getPIDController();

    public IntakeRollerSubsystem()
    {
        // setting the PID controllers properties
        double kI = 0,kP = 0,kD = 0,kFF = 0;
        rollerMotorPidController.setP(kP);
        rollerMotorPidController.setI(kI);
        rollerMotorPidController.setD(kD);
        rollerMotorPidController.setFF(kFF);
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

    // the subsystems commands
    // public Command OuttakeNoteCommand = Commands.startEnd(() -> this.setMotor(-1), () -> this.stopMotor(), this).until(() -> );
}
