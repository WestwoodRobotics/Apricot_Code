package frc.robot.commands.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    
    NeoTrajectory neoTrajectory = new NeoTrajectory(2,0,30);

    NeoTrajectory neoTrajectory2 = new NeoTrajectory(0, 0, 0);
    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(neoTrajectory.generateTrajectory().getInitialPose());

    // Run path following command, then stop at the end.
    return neoTrajectory.generateCommand(m_robotDrive).
          andThen(
           neoTrajectory2.generateCommand(m_robotDrive)
          ).andThen(
          () -> m_robotDrive.drive(0, 0, 0, false, false)
          );
  }
}