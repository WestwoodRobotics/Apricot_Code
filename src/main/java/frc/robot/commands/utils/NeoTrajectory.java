package frc.robot.commands.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.DriveSubsystem;

import java.util.ArrayList;

public class NeoTrajectory {
    private static Translation2d lastEnd = new Translation2d(0, 0);
    private Translation2d start;
    private Translation2d end;
    private double rot;
    private TrajectoryConfig config;
    private boolean isReversed;

    public NeoTrajectory(double x, double y, double rot) {
        this.start = lastEnd;
        this.end = new Translation2d(x, y);
        this.rot = rot;

        isReversed = start.getX() > end.getX();

        config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(isReversed);

        lastEnd = end; // Update the last end position
    }

    public Trajectory generateTrajectory() {
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(start);
        waypoints.add(end);

        ArrayList<Pose2d> poses = new ArrayList<>();

        for (Translation2d waypoint : waypoints) {
            poses.add(new Pose2d(waypoint, new Rotation2d(0)));
        }

        return TrajectoryGenerator.generateTrajectory(poses, config);
    }

    public SwerveControllerCommand generateCommand(DriveSubsystem m_robotDrive) {
        Trajectory trajectory = this.generateTrajectory();
        SwerveControllerCommand command = new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
            new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
                AutoConstants.kThetaControllerConstraints),
            () -> new Rotation2d(Math.toRadians(this.getRot())),
            m_robotDrive::setModuleStates,
            m_robotDrive
        );
        return command;
    }

    public double getRot() {
        return rot;
    }
}