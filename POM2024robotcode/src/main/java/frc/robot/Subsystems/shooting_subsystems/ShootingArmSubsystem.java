package frc.robot.Subsystems.shooting_subsystems;

import static frc.robot.Constants.ShootingConstants.*;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingArmSubsystem extends PomSubsystem{

  enum State{
    TakeFromIntake(0),
    OpenForIntake(1),
    AutoForShoot(2),
    AMP(3),
    ShootFromPodium(4),
    Unkown(5);
    public final int value;

    State(int value) {
      this.value = value;
    }
  
  }

    private State state = State.Unkown;
    private final CANSparkMax liftMotor;
    private final CANSparkMax liftMotorSlave;
    private final RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private DigitalInput brakeMicroSwitch;
    private BooleanSupplier intakeIsThere;

    private final ProfiledPIDController controller;
  /** Creates a new LiftSubsystem. */
  public ShootingArmSubsystem() {
    controller = new ProfiledPIDController (KP, KI, KD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_RAD_PER_SECOND, MAX_ACCELERATION_RAD_PER_SECOND_SQUARED), 0.02);
    controller.setTolerance(TOLERANCE);

    liftMotor = new CANSparkMax(SHOOTER_ARM_MOTOR, MotorType.kBrushless);
    liftMotorSlave = new CANSparkMax(14, MotorType.kBrushless);
    liftMotor.setInverted(true);
    encoder = liftMotor.getEncoder();
    pid = liftMotor.getPIDController();
    
    encoder.setPositionConversionFactor(POSITON_FACTOR);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR);

    liftMotorSlave.getEncoder().setPositionConversionFactor(POSITON_FACTOR);
    liftMotorSlave.getEncoder().setVelocityConversionFactor(VELOCITY_FACTOR);
    liftMotorSlave.setIdleMode(IdleMode.kBrake);
    liftMotorSlave.follow(liftMotor, true);
    pid.setP(KP, 0);
    pid.setI(KI, 0);
    pid.setD(KD, 0);
    pid.setIZone(KIZONE, 0);
    foldMicroSwitch = new DigitalInput(FOLD_MICRO_SWITCH_ID);
    brakeMicroSwitch = new DigitalInput(BRAKE_MICRO_SWITCH_ID);

    liftMotor.setIdleMode(IdleMode.kBrake); 


    // setDefaultCommand(stayInPlaceCommand());
    setDefaultCommand(runOnce(() -> stopMotor()));
    // resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isFoldSwitchPressed())
    {
      resetEncoder();
    }

    SmartDashboard.putNumber("arm encoder", getEncoderPosition());
    SmartDashboard.putNumber("arm motor", liftMotor.get());
    SmartDashboard.putNumber("arm set point", controller.getGoal().position);
    SmartDashboard.putBoolean("arm micro switch", isFoldSwitchPressed());
    SmartDashboard.putBoolean("arm can't move", intakeIsThere.getAsBoolean());
    SmartDashboard.putBoolean("brake switch", !brakeMicroSwitch.get());
    
    if (controller.getSetpoint().position == INTAKE_CAN_MOVE)
      state = State.OpenForIntake;
    else if (controller.getSetpoint().position == SUB_INTAKE_POS)
      state = State.TakeFromIntake;
    else if (controller.getSetpoint().position == SHOOT_PODIUM_POS)
      state = State.ShootFromPodium;
    else if (controller.getSetpoint().position == SHOOT_AMP_POS)
      state = State.AMP;
    else
      state = State.Unkown;
    liftMotor.setIdleMode(brakeMicroSwitch.get() ? IdleMode.kBrake : IdleMode.kCoast);
    liftMotorSlave.setIdleMode(brakeMicroSwitch.get() ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setIntakeSup(BooleanSupplier sup)
  {
    intakeIsThere = sup;
  }

  /** returns is the fold switch pressed
   * @return the fold limit switch value
   */
  public boolean isFoldSwitchPressed(){
    return !foldMicroSwitch.get();
  }

  public State getState()
  {
    return state;
  }

  public boolean atSetpoint()
  {
    return controller.atSetpoint();
  }

  /** Resets the encoder to currently read a position of 0. */
  @Override
  public void resetEncoder() {
    liftMotor.getEncoder().setPosition(ARM_OFFSET);
  }

  /** returns the value of the alternate encoder
   * @return the encoder position
   */
  @Override
  public double getEncoderPosition()
  {
    return encoder.getPosition();
  }

  
  public RelativeEncoder getEncoder(){
    return encoder;
  }

  public void moveWithProfile(TrapezoidProfile.State goal)
  {
    controller.setGoal(goal);
    if(isFoldSwitchPressed() && goal.position == SUB_INTAKE_POS)
    {
      stopMotor();
      SmartDashboard.putBoolean("Enteredd", false);
    }
    else{
      SmartDashboard.putBoolean("Enteredd", true);
      setMotor(controller.calculate(encoder.getPosition()));
    }
  }

  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  @Override
  public void setSetPoint(double target) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition, 0);
  }

  /**sets the motor to a paramater value
  * @param power the power to set the motor to
  */
  @Override
  public void setMotor(double speed)
  {
    liftMotor.set(speed);
  }

  public void setVoltage(double volts)
  {
    liftMotor.setVoltage(volts);
  }

  @Override
  public void stopMotor()
  {
    liftMotor.set(0);
  }

  public CANSparkMax getMotor(){
    return liftMotor;
  }

  public BooleanSupplier intakeCantMove()
  {
    return () -> !(encoder.getPosition() >= INTAKE_CAN_MOVE && controller.getSetpoint().position > INTAKE_CAN_MOVE);
  }

  public double calcArmPosForShoot(Supplier<Pose2d> curr)
  {
    Pose2d pose = curr.get();
    if(pose.getX() > OTHER_SIDE)
    {
      return ARM_OFFSET;
    }
    double k = Math.sqrt(Math.pow(pose.getX(), 2.0) + Math.pow(pose.getY() - SPEAKER_Y_OFFSET, 2.0));
    double alpha = SHOOTER_ANGLE_TO_ARM + 
                  Math.asin(
                    (ARM_LENGTH * Math.sin(SHOOTER_ANGLE_TO_ARM)) /
                    (Math.sqrt(k*k + Math.pow(SPEAKER_HEIGT_WANTED - ARM_HEIGT_FROM_FLOOR, 2)))
                    ) - 
                  Math.atan((SPEAKER_HEIGT_WANTED - ARM_HEIGT_FROM_FLOOR) / k);
    return alpha + ARM_OFFSET;
  }

  public Command goToAngleCommand(TrapezoidProfile.State goal)
  {
    return runOnce(()-> controller.reset(getEncoderPosition())).andThen((this.run(() -> moveWithProfile(goal)).until(()-> controller.atGoal())).unless(() -> goal.position < INTAKE_CAN_MOVE && intakeIsThere.getAsBoolean()));
  }
  public Command goToAngleCommand(double goal)
  {
    return Commands.print("goToAngleCommand").andThen(goToAngleCommand(new TrapezoidProfile.State(goal, 0)));
  }
  public Command goToAngleCommand(Supplier<Pose2d> poseSup)
  {
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(new TrapezoidProfile.State(calcArmPosForShoot(poseSup), 0))).until(()-> controller.atGoal()).unless(() -> calcArmPosForShoot(poseSup) < INTAKE_CAN_MOVE && intakeIsThere.getAsBoolean()));
  }
    public Command stayInPlaceCommand()
  {
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen((this.run(() -> moveWithProfile(controller.getGoal())).until(()-> controller.atGoal())));
  }
  public Command OpenForIntakeCommand()
  {
    return Commands.print("OpenForIntakeCommand").andThen(runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(new TrapezoidProfile.State(INTAKE_CAN_MOVE + 0.2, 0))).until(()-> encoder.getPosition() > INTAKE_CAN_MOVE).andThen(() -> stopMotor())));
  }
  public Command closeSlow()
  {
      return run(() -> setMotor(-0.27)).until(()-> isFoldSwitchPressed()).unless(this::isFoldSwitchPressed);
  }
  public Command joystickShootCommand(DoubleSupplier sup)
  {
    // return run(() -> setMotor(Math.abs(sup.getAsDouble()) > 0.6 ? Math.copySign(0.6, sup.getAsDouble()) : sup.getAsDouble()));
    return run(() -> setMotor(sup.getAsDouble()));
  }

  public Command Climb()
  {
    return startEnd(() -> setMotor(-0.4), this::stopMotor).until(this::isFoldSwitchPressed);
  }

}