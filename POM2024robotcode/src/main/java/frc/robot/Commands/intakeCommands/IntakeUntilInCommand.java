package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class IntakeUntilInCommand extends Command
{
     IntakeRollerSubsystem intakeRollerSubsystem;

    public IntakeUntilInCommand(IntakeRollerSubsystem intakeRollerSubsystem)
    {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(intakeRollerSubsystem);

    }

    @Override
    public void initialize()
    {
        intakeRollerSubsystem.setMotor(1);

    }

    @Override
    public void end(boolean interrupted)
    {
        intakeRollerSubsystem.stopMotor();

    }

    @Override
    public boolean isFinished()
    {
        return intakeRollerSubsystem.isNoteIn();
    }
}
