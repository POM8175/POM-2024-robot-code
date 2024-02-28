package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ArmLockConstants;

public class ArmLockSubsystem extends PomSubsystem
{
    Servo lockServo = new Servo(ArmLockConstants.lockServoChanel);
    double startPos;

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
}
