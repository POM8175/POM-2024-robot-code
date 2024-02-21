// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import static frc.robot.Constants.DriveConstants.ANGLE_TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class TurnByDegreeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private final PIDController anglePidController;
  private Rotation2d mSetPoint;
  private double mDelta;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public  TurnByDegreeCommand(DriveSubsystem subsystem, double delta) {
    driveSubsystem = subsystem;    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    mDelta = delta;
    SmartDashboard.putNumber("angle kp", KP);
    SmartDashboard.putNumber("angle ki", KI);
    SmartDashboard.putNumber("angle kd", KD);
    SmartDashboard.putNumber("mDelta", KD);
    
    
    anglePidController = new PIDController(KP, KI, KD);
    anglePidController.enableContinuousInput(-180, 180);
    anglePidController.setTolerance(ANGLE_TOLERANCE);
  }
  
  double KP = 0;
  double KI = 0.0;
  double KD = 0.0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    KP = SmartDashboard.getNumber("angle kp", KP);
    KI = SmartDashboard.getNumber("angle ki", KI);
    KD = SmartDashboard.getNumber("angle kd", KD);
    anglePidController.reset();
    mDelta = SmartDashboard.getNumber("mDelta", 0);
    mSetPoint =  Rotation2d.fromDegrees(((int)driveSubsystem.getHeading()) % 360).plus(Rotation2d.fromDegrees(mDelta));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    anglePidController.setSetpoint(mSetPoint.getDegrees());
    double r = anglePidController.calculate(((int)driveSubsystem.getHeading()) % 360);
    driveSubsystem.arcadeDrive(0, r > 0.5 ? 0.5 : r);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePidController.atSetpoint();
  }
}
