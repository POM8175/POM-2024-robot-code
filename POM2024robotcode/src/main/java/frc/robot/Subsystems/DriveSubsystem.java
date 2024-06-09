package frc.robot.Subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.TrajectoryFactory;
/**
 *
 */
public class DriveSubsystem extends PomSubsystem {

  private Field2d field;

  private final WPI_VictorSPX masterRightMotor = new WPI_VictorSPX(RIGHT_MOTOR_LEAD);
  private final WPI_VictorSPX slaveRightMotor = new WPI_VictorSPX(RIGHT_MOTOR_SLAVE);

  private final WPI_VictorSPX masterLeftMotor = new WPI_VictorSPX(LEFT_MOTOR_LEAD);
  private final WPI_VictorSPX slaveLeftMotor = new WPI_VictorSPX(LEFT_MOTOR_SLAVE);

  // private final SparkPIDController leftPid = masterLeftMotor.getPIDController();
  // private final SparkPIDController rightPid = masterRightMotor.getPIDController();

  // private final RelativeEncoder leftEncoder;
  // private final RelativeEncoder rightEncoder;

  private final DifferentialDrive mDrive = new DifferentialDrive(masterLeftMotor::set, masterRightMotor::set);

  private final WPI_PigeonIMU mGyro = new WPI_PigeonIMU(GYRO_ID);

  double x = 0,y = 0;
  boolean isNote = false;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    zeroHeading();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    masterRightMotor.setInverted(true);
    slaveRightMotor.setInverted(true);
    masterLeftMotor.setInverted(false);
    slaveLeftMotor.setInverted(false);
    
    
    field = new Field2d();


    SmartDashboard.putData("Field", field);

    // slaveLeftMotor.follow(masterLeftMotor);
    // slaveRightMotor.follow(masterRightMotor);

    masterLeftMotor.setNeutralMode(NeutralMode.Coast);
    masterRightMotor.setNeutralMode(NeutralMode.Coast);
    slaveLeftMotor.setNeutralMode(NeutralMode.Coast);
    slaveRightMotor.setNeutralMode(NeutralMode.Coast);
    mGyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    field.setRobotPose(getPose());
    
    SmartDashboard.putNumber("left Drive", masterLeftMotor.get());
    SmartDashboard.putNumber("right Drive", masterRightMotor.get());

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
    return new Pose2d();
  }
  public Pose2d getPoseOdometry() {
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
    return new DifferentialDriveWheelSpeeds(0,0);
  }

  double output;
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    //restroe if slew rate doesnt work properly
    if (Math.abs(output) < Math.abs(fwd / 2)) {
      output = fwd / 2;
    }
    if (fwd - output > RATE) {
      output += RATE;
    } else if (output - fwd > RATE) {
      output -= RATE;
    }
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("rot", rot);
    mDrive.arcadeDrive(output, rot);
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
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return 0;
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
    return mGyro.getFusedHeading() % 360;
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
    return this.run(() -> tankDriveVolts(left.get(), right.get()));
  }


  boolean isReverse = false;
  boolean lastReverse = false;
  public Command arcadeDriveCommand(Supplier<Double> fwd, Supplier<Double> rot, BooleanSupplier reverse)
  {
    // SlewRateLimiter rateLimit = new SlewRateLimiter(RATE);
    // SlewRateLimiter turnRateLimit = new SlewRateLimiter(RATE);
    // rateLimit.reset((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2);
    // return this.run(() -> arcadeDrive(rateLimit.calculate(-fwd.get()), turnRateLimit.calculate(rot.get())));
    return this.run(() -> {
      double f = fwd.get();
      if(reverse.getAsBoolean() && !lastReverse) { isReverse = !isReverse;}
      lastReverse = reverse.getAsBoolean();
      if(isReverse){ f = -f;}
      arcadeDrive((-f), (rot.get()));
    });
    
  }

  public double calcAngleToSpeaker()
  {
    try{
      return Math.atan((getPose().getY() - SPEAKER_Y) / 
      (DriverStation.getAlliance().get() == Alliance.Red ? getPose().getX() : FIELD_X - getPose().getX()));
    }
    catch(Exception e)
    {
      return 0;

    }
  }

  public Trajectory driveToNoteTrajectory()
  {

    Pose2d notePose = getPose().transformBy(new Transform2d(x,y, new Rotation2d(-Math.atan(x/y))));
    field.getObject("note").setPose(notePose);

    Trajectory t = TrajectoryFactory.trajectoryFactory(getPose(), new ArrayList<Translation2d>(), notePose, false);
    field.getObject("traj").setTrajectory(t);
    return t;
  }
}

