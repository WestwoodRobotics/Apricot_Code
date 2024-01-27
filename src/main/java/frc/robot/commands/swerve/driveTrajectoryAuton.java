package frc.robot.commands.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.DriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;



public class driveTrajectoryAuton extends CommandBase {

  private final DriveSubsystem m_robotDrive;

  public driveTrajectoryAuton(DriveSubsystem swerveDrive) {
    m_robotDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);


    // Define points directly
    ArrayList<Pose2d> allWaypoints = new ArrayList<>();
    allWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
    allWaypoints.add(new Pose2d(2, 0, new Rotation2d(Math.toRadians(0))));
    // allWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));

    ArrayList<Pose2d> allWaypoints2 = new ArrayList<>();
    allWaypoints2.add(new Pose2d(2, 0, new Rotation2d(Math.toRadians(0))));
    allWaypoints2.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));

    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      allWaypoints,
      config
    );
    Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
      allWaypoints2,
      config.setReversed(true)
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        xController,
        yController,
        thetaController,
        () -> new Rotation2d(Math.toRadians(30)),
        m_robotDrive::setModuleStates,
        m_robotDrive);
        
      SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        exampleTrajectory2,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        xController,
        yController,
        thetaController,
        () -> new Rotation2d(Math.toRadians(0)),
        m_robotDrive::setModuleStates,
        m_robotDrive);
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(
      swerveControllerCommand2
    ).andThen(
      () -> m_robotDrive.drive(
        0, 0, 0, false, false
      )
    );

  }
}