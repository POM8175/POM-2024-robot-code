package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import static frc.robot.Constants.DriveConstants.*;

public class TrajectoryFactory {
    private static TrajectoryConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                KS_VOLTS,
                KV_VOLT_SECOND_PER_METER,
                KA_VOLT_SECONDS_SQUARE_PER_METER),
            DRIVE_KINEMATICS,
            10);

    private static TrajectoryConfig tConfig = new TrajectoryConfig(MAX_SPEED_METER_PER_SECOND,
     MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
     ).setKinematics(DRIVE_KINEMATICS).addConstraint(autoVoltageConstraint);

     /**
      * creates a drive trajectory based on the params
      * @param start the point that the robot starts at
      * @param innerPoints list of points that the robot should go through
      * @param end the point that the robot end at
      * @param reversed should the robot drive backward
      * @return the generated trajectory
      */
    public static Trajectory trajectoryFactory(Pose2d start, ArrayList<Translation2d> innerPoints, Pose2d end, boolean reversed){
        tConfig.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, innerPoints, end, tConfig);
    }

}
