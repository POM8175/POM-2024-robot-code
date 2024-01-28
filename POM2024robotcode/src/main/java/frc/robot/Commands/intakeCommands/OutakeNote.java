package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class OutakeNote extends Command
{
    IntakeRollerSubsystem rollerSubsystem;

    public OutakeNote(IntakeRollerSubsystem rollerSubsystem)
    {
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize()
    {
        rollerSubsystem.setMotor(-1);
    }

    @Override
    public void end(boolean interrupted)
    {
        rollerSubsystem.stopMotor();;
    }

    @Override
    public boolean isFinished()
    {
        return !rollerSubsystem.isNoteIn();
    }
}
