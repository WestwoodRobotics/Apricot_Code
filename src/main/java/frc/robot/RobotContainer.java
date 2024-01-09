
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.Intake.IntakeCommand;
//import frc.robot.commands.elevator.ElevatorCommand;
//import frc.robot.commands.elevator.ElevatorPosSet;
import frc.robot.commands.swerve.driveCommand;
//import frc.robot.commands.wrist.WristCommand;
//import frc.robot.commands.wrist.WristPosSet;
//import frc.robot.subsystems.elevator.ElevatorModule;
//import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.DriveSubsystem;
//import frc.robot.subsystems.wrist.WristModule;
import frc.robot.subsystems.test.Test;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Test test = new Test();
  // private final IntakeModule m_intakeModule = new IntakeModule();
  // private final ElevatorModule m_elevatorModule = new ElevatorModule();
  // private final WristModule m_wristModule = new WristModule();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  
  private final JoystickButton yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

  // private final POVButton dPadUp = new POVButton(m_driverController, 0);
  // private final POVButton dPadRight = new POVButton(m_driverController, 90);
  // private final POVButton dPadDown = new POVButton(m_driverController, 180);
  // private final POVButton dPadLeft = new POVButton(m_driverController, 270);

  private final JoystickButton rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);


  // private final JoystickButton y2Button = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  // private final JoystickButton a2Button = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  // private final JoystickButton b2Button = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  // private final JoystickButton x2Button = new JoystickButton(m_operatorController, XboxController.Button.kX.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    // Configure default commands 
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    test.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    // m_intakeModule.setDefaultCommand(new IntakeCommand(m_intakeModule, m_driverController, m_operatorController));
    // m_elevatorModule.setDefaultCommand(new ElevatorCommand(m_elevatorModule, m_driverController, m_operatorController));
    // m_wristModule.setDefaultCommand(new WristCommand(m_wristModule, m_driverController, m_operatorController));

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
   new JoystickButton(m_operatorController, Button.kR1.value) // if R1 is pressed wheels should go into x formation
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.resetGyro(),
            m_robotDrive));



    // dPadUp.whileTrue(new InstantCommand(() -> m_elevatorModule.setElevatorPower(-0.25)));
    // dPadDown.whileTrue(new InstantCommand(() -> m_elevatorModule.setElevatorPower(0.25)));
    // dPadLeft.whileTrue(new InstantCommand(() -> m_wristModule.setArmPower(-0.25)));
    // dPadRight.whileTrue(new InstantCommand(() -> m_wristModule.setArmPower(0.25)));

    // leftBumper.onTrue(new ElevatorPosSet(m_elevatorModule, "cube_pickup")
    //           .andThen(new WristPosSet(m_wristModule, "cube_pickup")));
    // rightBumper.onTrue(new ElevatorPosSet(m_elevatorModule, "cone_pickup")
    //           .andThen(new WristPosSet(m_wristModule, "cone_pickup")));

    // aButton.onTrue(new ElevatorPosSet(m_elevatorModule, "cube_pickup")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));
    // yButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cone_high/cube_mid")));
    // bButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));
    // xButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));


    //a2Button.onTrue(new ElevatorPosSet(m_elevatorModule, "home/low_cube")
    //           .andThen(new WristPosSet(m_wristModule, "home/low_cube")));  

    
    
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

    // Get points from Smart Dashboard
    // Get number of waypoints from Smart Dashboard
    int numberOfWaypoints = (int) SmartDashboard.getNumber("Number of Waypoints", 0);

    // Get points from Smart Dashboard
    List<Translation2d> interiorWaypoints = new ArrayList<>();
    for (int i = 0; i < numberOfWaypoints; i++) {
        double x = SmartDashboard.getNumber("Waypoint " + i + " X", 0);
        double y = SmartDashboard.getNumber("Waypoint " + i + " Y", 0);
        interiorWaypoints.add(new Translation2d(x, y));
    }

    // Generate trajectory
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0.0)),
        // Pass through the interior waypoints
        interiorWaypoints,
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0.01)),
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