package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class OutakeNote extends Command
{
    IntakeSubsystem subsystem;

    public OutakeNote(IntakeSubsystem subsystem)
    {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        subsystem.rollerSubsystem.setMotor(-1);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.rollerSubsystem.stopMotor();;
    }

    @Override
    public boolean isFinished()
    {
        return !subsystem.isNoteIn();
    }
}
