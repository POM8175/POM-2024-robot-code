package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends PomSubsystem
{
    VictorSPX transferMotor = new VictorSPX(0);

    public TransferSubsystem()
    {

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
        return new StartEndCommand(() -> setMotor(1), () -> stopMotor(), this);
    }
    public Command transferNote()
    {
        return new StartEndCommand(() -> setMotor(1), () -> stopMotor(), this);
    }
}
