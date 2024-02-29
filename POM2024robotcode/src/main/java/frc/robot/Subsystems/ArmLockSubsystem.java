package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static frc.robot.Constants.ArmLockConstants.*;
public class ArmLockSubsystem extends PomSubsystem
{
    Servo lockServo = new Servo(lockServoChanel);
    double startPos;

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Lock Servo", lockServo.get());
        SmartDashboard.putNumber("Arm Lock Servo Angle", lockServo.getAngle());
        SmartDashboard.putNumber("Arm Lock Servo Position", lockServo.getPosition());
        SmartDashboard.putNumber("Arm Lock startPos", startPos);
    }

    public ArmLockSubsystem()
    {
        startPos = lockServo.getAngle();
    }

    public void lockServo()
    {
        lockServo.setAngle(startPos+90);
    } 

    public void openServo()
    {
        lockServo.setAngle(startPos);
    }

    public Command TurnTo(double angle)
    {
        return new InstantCommand(() -> lockServo.setAngle(angle));
    }
}
