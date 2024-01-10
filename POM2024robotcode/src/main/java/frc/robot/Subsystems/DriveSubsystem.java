package frc.robot.Subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 *
 */
public class DriveSubsystem extends PomSubsystem {

  private double output = 0;

  private Field2d field;

  private final CANSparkMax masterRightMotor = new CANSparkMax(RIGHT_MOTOR_LEAD, MotorType.kBrushless);
  private final CANSparkMax slaveRightMotor = new CANSparkMax(RIGHT_MOTOR_SLAVE, MotorType.kBrushless);

  private final CANSparkMax masterLeftMotor = new CANSparkMax(LEFT_MOTOR_LEAD, MotorType.kBrushless);
  private final CANSparkMax slaveLeftMotor = new CANSparkMax(LEFT_MOTOR_SLAVE, MotorType.kBrushless);

  private final SparkPIDController leftPid = masterLeftMotor.getPIDController();
  private final SparkPIDController rightPid = masterRightMotor.getPIDController();

  private final RelativeEncoder leftEncoder = masterLeftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = masterRightMotor.getEncoder();

  private final DifferentialDrive mDrive = new DifferentialDrive(masterLeftMotor, masterRightMotor);

  private final WPI_PigeonIMU mGyro = new WPI_PigeonIMU(GYRO_ID);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    leftPid.setP(KP);
    leftPid.setI(KI);
    leftPid.setD(0);

    rightPid.setP(KP);
    rightPid.setI(KI);
    rightPid.setD(0);

    zeroHeading();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    masterRightMotor.setInverted(true);

    leftEncoder.setPositionConversionFactor(ROTATIONS_TO_METERS);
    rightEncoder.setPositionConversionFactor(ROTATIONS_TO_METERS);

    field = new Field2d();


    SmartDashboard.putData("Field", field);

    slaveLeftMotor.follow(masterLeftMotor);
    slaveRightMotor.follow(masterRightMotor);

    masterLeftMotor.setIdleMode(IdleMode.kCoast);
    masterRightMotor.setIdleMode(IdleMode.kCoast);
    slaveLeftMotor.setIdleMode(IdleMode.kCoast);
    slaveRightMotor.setIdleMode(IdleMode.kCoast);
    
    mGyro.reset();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  @Override
  public void stopMotor(){
    masterLeftMotor.stopMotor();
    masterRightMotor.stopMotor();
  }

  /**
   * sets the mottors to go a distance from the curent position.
   * @param targetPosition the distance to drive
   */
  @Override
  public void setSetPoint(double distance) {
    leftPid.setReference(getLeftEncoder().getPosition() + distance, CANSparkMax.ControlType.kPosition);
    rightPid.setReference(getLeftEncoder().getPosition() + distance, CANSparkMax.ControlType.kPosition);
  }

  /** returns the current pitch of the robot from gyro
   * @return the pitch angle
  */
  public double getPitchAngle() {
    return mGyro.getPitch();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //TODO: return to a real position when implementing odometry/poseestimator
    //return estimator.getEstimatedPosition();
    return new Pose2d();
  }

  /**sets all of the motors to a paramater value
   * @param speed the power to set the motors to
  */
  @Override
  public void setMotor(double speed) {
    masterLeftMotor.set(speed);
    masterRightMotor.set(speed);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(masterLeftMotor.getEncoder().getVelocity(),
        masterRightMotor.getEncoder().getVelocity());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    if (Math.abs(output) < Math.abs(fwd / 2)) {
      output = fwd / 2;
    }
    if (fwd - output > RATE) {
      output += RATE;
    } else if (output - fwd > RATE) {
      output -= RATE;
    }

    mDrive.arcadeDrive(output, rot);
  }
  public void simpleArcadeDrive(double fwd, double rot){
    mDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    masterLeftMotor.setVoltage(leftVolts);
    masterRightMotor.setVoltage(rightVolts);
    mDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  @Override
  public void resetEncoder() {
    masterLeftMotor.getEncoder().setPosition(0);
    masterRightMotor.getEncoder().setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (masterLeftMotor.getEncoder().getPosition()
        + masterRightMotor.getEncoder().getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return masterRightMotor.getEncoder();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return masterRightMotor.getEncoder();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    mDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.setFusedHeading(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return mGyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -mGyro.getRate();
  }


  public Command tankDriveCommand(Supplier<Double> left, Supplier<Double> right)
  {
    return new RunCommand(() -> tankDriveVolts(left.get(), right.get()), this);
  }


    public Command arcadeDriveCommand(Supplier<Double> left, Supplier<Double> right)
  {
    return new RunCommand(() -> arcadeDrive(left.get(), right.get()), this);
  }



}

