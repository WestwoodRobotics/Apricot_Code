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
import frc.robot.commands.utils.NeoTrajectory;

import edu.wpi.first.wpilibj.XboxController;



public class driveTrajectoryAuton extends CommandBase {

  private final DriveSubsystem m_robotDrive;

  public driveTrajectoryAuton(DriveSubsystem swerveDrive) {
    m_robotDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  public Command getAutonomousCommand() {

    NeoTrajectory neoTrajectory = new NeoTrajectory(new Pose2d(2, 0, new Rotation2d(Math.toRadians(0))), 30);

    NeoTrajectory neoTrajectory2 = new NeoTrajectory(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), 0);

    SwerveControllerCommand swerveControllerCommand = neoTrajectory.generateCommand(m_robotDrive);

    SwerveControllerCommand swerveControllerCommand2 = neoTrajectory2.generateCommand(m_robotDrive);
    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(neoTrajectory.generateTrajectory().getInitialPose());

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