package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class IntakeUntilInCommand extends Command
{
     IntakeRollerSubsystem intakeRollerSubsystem;

     double motorInputPower;

    public IntakeUntilInCommand(IntakeRollerSubsystem intakeRollerSubsystem, double speed)
    {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(intakeRollerSubsystem);
        motorInputPower = speed;
    }

    @Override
    public void initialize()
    {
        intakeRollerSubsystem.setMotor(motorInputPower);

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
