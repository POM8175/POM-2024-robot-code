package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class OutakeNote extends Command
{
    IntakeRollerSubsystem rollerSubsystem;

    double motorOutputPower;

    public OutakeNote(IntakeRollerSubsystem rollerSubsystem,double speed)
    {
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
        motorOutputPower = speed;
    }

    @Override
    public void initialize()
    {
        rollerSubsystem.setMotor(motorOutputPower);
    }

    @Override
    public void end(boolean interrupted)
    {
        rollerSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished()
    {
        return !rollerSubsystem.isNoteIn();
    }
}
