// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveMeasured extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private final PIDController pid;
  private double meters;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public  DriveMeasured(DriveSubsystem subsystem, double meters) {
    driveSubsystem = subsystem;    
    this.meters = meters;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    pid = new PIDController(meters, meters, meters);
    SmartDashboard.putNumber("fwd kp", KP);
    SmartDashboard.putNumber("fwd ki", KI);
    SmartDashboard.putNumber("fwd kd", KD);
    SmartDashboard.putNumber("fwd meters", KD);
    pid.setTolerance(0.05);
  }
  double lEncoder;
  double KP = 0.5;
  double KI = 0.23;
  double KD = 0.0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    KP = SmartDashboard.getNumber("fwd kp", KP);
    KI = SmartDashboard.getNumber("fwd ki", KI);
    KD = SmartDashboard.getNumber("fwd kd", KD);
    meters = SmartDashboard.getNumber("fwd meters", 0);
    lEncoder = driveSubsystem.getLeftEncoder().getPosition();
    pid.setPID(KP, KI, KD);
    pid.setSetpoint(meters + lEncoder);
  }

  @Override
  public void execute()
  {
    double x = pid.calculate(driveSubsystem.getLeftEncoder().getPosition());
    driveSubsystem.arcadeDrive(x > 0.5 ? 0.5 : x, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
