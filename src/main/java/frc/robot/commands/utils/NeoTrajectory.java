package frc.robot.commands.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.*;

import java.util.ArrayList;

public class NeoTrajectory {
    private Pose2d start;
    private Pose2d end;
    private double rot;
    private TrajectoryConfig config;
    private boolean isReversed;

    public NeoTrajectory(Pose2d start, Pose2d end, double rot) {
        this.start = start;
        this.end = end;
        this.rot = rot;

        isReversed = start.getTranslation().getX() > end.getTranslation().getX();

        config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(isReversed);
    }

    public Trajectory generateTrajectory() {
        ArrayList<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(start);
        waypoints.add(end);
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    public double getRot() {
        return rot;
    }
}