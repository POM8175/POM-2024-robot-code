package frc.robot.Subsystems.shooting_subsystems;

import static frc.robot.Constants.ShootingConstants.*;


import java.util.function.BooleanSupplier;
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
import frc.robot.Subsystems.PomSubsystem;

public class ShootingArmSubsystem extends PomSubsystem{

  enum State{
    OpenForIntake,
    TakeFromIntake,
    AutoForShoot,
    AMP,
    ShootFromPodium,
    Unkown
  }

    private State state = State.Unkown;
    private final CANSparkMax liftMotor;
    private final RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private BooleanSupplier intakeIsThere;

    private final ProfiledPIDController controller;
  /** Creates a new LiftSubsystem. */
  public ShootingArmSubsystem() {
    controller = new ProfiledPIDController (KP, KI, KD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_RAD_PER_SECOND, MAX_ACCELERATION_RAD_PER_SECOND_SQUARED), 0.02);
    controller.setTolerance(TOLERANCE);

    liftMotor = new CANSparkMax(SHOOTER_ARM_MOTOR, MotorType.kBrushless);
    liftMotor.setInverted(true);
    encoder = liftMotor.getEncoder();
    pid = liftMotor.getPIDController();
    
    encoder.setPositionConversionFactor(POSITON_FACTOR);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR);
    pid.setP(KP, 0);
    pid.setI(KI, 0);
    pid.setD(KD, 0);
    pid.setIZone(KIZONE, 0);
    foldMicroSwitch = new DigitalInput(FOLD_MICRO_SWITCH_ID);

    liftMotor.setIdleMode(IdleMode.kCoast); // check


    setDefaultCommand(goToAngleCommand(controller.getGoal()));
    // setDefaultCommand(runOnce(() -> stopMotor()));
    resetEncoder();
  }

  @Override
  public void periodic() {
    //liftTab.add("arm encoder", liftMotor.getEncoder().getPosition()).withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider);
    // This method will be called once per scheduler run
    if(isFoldSwitchPressed())
    {
      resetEncoder();
    }

    SmartDashboard.putNumber("arm encoder", getEncoderPosition());
    SmartDashboard.putNumber("arm motor", liftMotor.get());
    SmartDashboard.putBoolean("arm micro switch", isFoldSwitchPressed());
    
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
    if(isFoldSwitchPressed() && state == State.TakeFromIntake)
    {
      stopMotor();
    }
    else{
      setMotor(controller.calculate(encoder.getPosition(), goal));
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

  public BooleanSupplier intakeCanMove()
  {
    return () -> encoder.getPosition() >= INTAKE_CAN_MOVE && controller.getSetpoint().position > INTAKE_CAN_MOVE;
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
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(goal)).until(()-> controller.atGoal()).unless(() -> goal.position < INTAKE_CAN_MOVE && intakeIsThere.getAsBoolean()));
  }
  public Command goToAngleCommand(double goal)
  {
    return goToAngleCommand(new TrapezoidProfile.State(goal, 0));
  }
  public Command goToAngleCommand(Supplier<Pose2d> poseSup)
  {
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(new TrapezoidProfile.State(calcArmPosForShoot(poseSup), 0))).until(()-> controller.atGoal()).unless(() -> calcArmPosForShoot(poseSup) < INTAKE_CAN_MOVE && intakeIsThere.getAsBoolean()));
  }
  public Command OpenForIntakeCommand()
  {
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(new TrapezoidProfile.State(INTAKE_CAN_MOVE, 0))).until(()-> encoder.getPosition() > INTAKE_CAN_MOVE).andThen(goToAngleCommand(new TrapezoidProfile.State(encoder.getPosition(), 0))));
  }

  public Command testtt(double goal)
  {
    return runOnce(() -> controller.reset(getEncoderPosition())).andThen(this.run(() -> moveWithProfile(new TrapezoidProfile.State(goal, 0))));
  }
}
