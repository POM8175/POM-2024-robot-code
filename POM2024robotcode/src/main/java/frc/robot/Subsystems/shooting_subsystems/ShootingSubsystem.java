package frc.robot.Subsystems.shooting_subsystems;


import static frc.robot.Constants.ShootingConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GeneralFunctions;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingSubsystem extends PomSubsystem {


    private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
    

    private final RelativeEncoder rightEncoder = shooterMotorRight.getEncoder();
    private final RelativeEncoder leftEncoder = shooterMotorLeft.getEncoder();

    private SparkPIDController leftPID = shooterMotorLeft.getPIDController();
    private SparkPIDController rightPID = shooterMotorRight.getPIDController();


   public ShootingSubsystem(){
       shooterMotorLeft.setIdleMode(IdleMode.kCoast);
       shooterMotorRight.setIdleMode(IdleMode.kCoast);
       shooterMotorLeft.setInverted(true);
       shooterMotorRight.setInverted(true);
       SmartDashboard.putNumber("wanted speed", 0);
       leftPID.setP(SHOOT_KP);
       rightPID.setP(SHOOT_KP);
       leftPID.setI(SHOOT_KI);
       rightPID.setI(SHOOT_KI);

   }

   @Override
   public void periodic() {
       SmartDashboard.putNumber("Shooting/Left Motor Speed", leftEncoder.getVelocity());
       SmartDashboard.putNumber("Shooting/Right Motor Speed", rightEncoder.getVelocity());
       SmartDashboard.putBoolean("Shooting/Is Follower", shooterMotorLeft.isFollower());
   }
   

   public boolean atWantedSpeed()
   {
    return GeneralFunctions.allowedError( getRate(), shooterMotorLeft.getAppliedOutput(), TOLERANCE);
   }

    @Override
    public void setMotor(double speed) {;
        rightPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        leftPID.setReference(1 * speed, CANSparkMax.ControlType.kVelocity);
    }

    public void setPercent(double percent) {
        shooterMotorLeft.set(percent);
        shooterMotorRight.set(percent);
    }
    @Override
    public void stopMotor() {
        shooterMotorRight.set(0);
        shooterMotorLeft.set(0);
    }

    
    public double getRate() {

        return (rightEncoder.getVelocity()+leftEncoder.getVelocity())/2;


    }

   public Command spinWheelsCommand()
    {
        return this.startEnd(() -> setMotor(SHOOT_SPEED), () -> {}).until(() -> getRate() >= SHOOT_SPEED / 2 - SHOOT_SPEED_TOLERANCE);
    }
    public Command spinWheelsToSpeedCommand(double speed)
    {
        return this.startEnd(() -> setMotor(speed), () -> {}).until(() -> getRate() >= speed - SHOOT_SPEED_TOLERANCE); 
    }

    public Command stopWheelsCommand()
    {
        return this.runOnce(() -> stopMotor());
    }

    public Command setPercentComand(double percent){
        return this.run(() -> setPercent(percent)); 
    }

    public Command joystickShootCommand(DoubleSupplier sup)
    {
        return run(() -> setMotor(sup.getAsDouble()));
    }
    public Command smartdashShootCommand()
    {
        return run(() -> setMotor(SmartDashboard.getNumber("wanted speed", 0)));
    }        

}
