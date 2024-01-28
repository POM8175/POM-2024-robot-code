package frc.robot.Subsystems.shooting_subsystems;


import static frc.robot.Constants.DriveConstants.ROTATIONS_TO_METERS;
import static frc.robot.Constants.ShootingConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Subsystems.PomSubsystem;

public class ShootingSubsystem extends PomSubsystem {


    private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
    

    private final RelativeEncoder rightEncoder = shooterMotorRight.getEncoder();
    private final RelativeEncoder leftEncoder = shooterMotorLeft.getEncoder();




    private void ShootingSubsystem(){

        rightEncoder.setVelocityConversionFactor(ROTATIONS_TO_METERS);
        leftEncoder.setPositionConversionFactor(ROTATIONS_TO_METERS);
        shooterMotorLeft.follow(shooterMotorRight, true);

    }

    @Override
    public void setMotor(double speed) {
        shooterMotorRight.set(speed);
    }

    public double getRate() {

        return (rightEncoder.getVelocity()+leftEncoder.getVelocity())/2;

    }


}
