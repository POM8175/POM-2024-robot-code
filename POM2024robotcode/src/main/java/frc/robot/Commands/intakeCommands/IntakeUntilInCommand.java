package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeUntilInCommand extends Command
{
    IntakeSubsystem intakeSubsystem;

    public IntakeUntilInCommand(IntakeSubsystem intakeSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        intakeSubsystem.rollerSubsystem.setMotor(1);
    }

    @Override
    public void end(boolean interrupted)
    {
        intakeSubsystem.rollerSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished()
    {
        return intakeSubsystem.isNoteIn();
    }
}
