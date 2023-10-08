
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.elevator.ElevatorPosSet;
import frc.robot.commands.swerve.driveCommand;
import frc.robot.commands.wrist.WristCommand;
import frc.robot.commands.wrist.WristPosSet;
import frc.robot.subsystems.elevator.ElevatorModule;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.wrist.WristModule;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeModule m_intakeModule = new IntakeModule();
  private final ElevatorModule m_elevatorModule = new ElevatorModule();
  private final WristModule m_armModule = new WristModule();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  
  private final JoystickButton yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

  private final POVButton dPadUp = new POVButton(m_driverController, 0);
  private final POVButton dPadRight = new POVButton(m_driverController, 90);
  private final POVButton dPadDown = new POVButton(m_driverController, 180);
  private final POVButton dPadLeft = new POVButton(m_driverController, 270);

  private final JoystickButton rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);


  private final JoystickButton y2Button = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton a2Button = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton b2Button = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton x2Button = new JoystickButton(m_operatorController, XboxController.Button.kX.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    m_intakeModule.setDefaultCommand(new IntakeCommand(m_intakeModule, m_driverController, m_operatorController));
    m_elevatorModule.setDefaultCommand(new ElevatorCommand(m_elevatorModule, m_driverController, m_operatorController));
    m_armModule.setDefaultCommand(new WristCommand(m_armModule, m_driverController, m_operatorController));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /*new JoystickButton(m_driverController, Button.kR1.value) // if R1 is pressed wheels should go into x formation
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));*/


    // dPadUp.whileTrue(new InstantCommand(() -> m_elevatorModule.setElevatorPower(-0.25)));
    // dPadDown.whileTrue(new InstantCommand(() -> m_elevatorModule.setElevatorPower(0.25)));
    // dPadLeft.whileTrue(new InstantCommand(() -> m_armModule.setArmPower(-0.25)));
    // dPadRight.whileTrue(new InstantCommand(() -> m_armModule.setArmPower(0.25)));

    // leftBumper.onTrue(new ElevatorPosSet(m_elevatorModule, "cube_pickup")
    //           .andThen(new ArmPosSet(m_armModule, "cube_pickup")));
    // rightBumper.onTrue(new ElevatorPosSet(m_elevatorModule, "cone_pickup")
    //           .andThen(new ArmPosSet(m_armModule, "cone_pickup")));

    // aButton.onTrue(new ElevatorPosSet(m_elevatorModule, "home/low_cube")
    //           .andThen(new ArmPosSet(m_armModule, "home/low_cube")));
    // yButton.onTrue(new ElevatorPosSet(m_elevatorModule, "cone_high/cube_mid")
    //           .andThen(new ArmPosSet(m_armModule, "cone_high/cube_mid")));
    // bButton.onTrue(new ElevatorPosSet(m_elevatorModule, "cone_mid")
    //           .andThen(new ArmPosSet(m_armModule, "cone_mid")));
    // xButton.onTrue(new ElevatorPosSet(m_elevatorModule, "cube_high")
    //           .andThen(new ArmPosSet(m_armModule, "cube_high")));


    // a2Button.onTrue(new ElevatorPosSet(m_elevatorModule, "home/low_cube")
    //           .andThen(new ArmPosSet(m_armModule, "home/low_cube")));  

    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
