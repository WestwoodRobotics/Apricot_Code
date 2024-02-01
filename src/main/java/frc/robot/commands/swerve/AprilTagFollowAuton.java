package frc.robot.commands.swerve;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.utils.NeoTrajectory;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;


public class AprilTagFollowAuton extends CommandBase{

    private final DriveSubsystem m_robotDrive;
    private Vision limelight;
    private double idealAprilTagArea;


    public AprilTagFollowAuton(DriveSubsystem m_robotDrive, Vision limelight, double idealArea) {
        this.limelight = limelight;
        this.m_robotDrive = m_robotDrive;
        this.idealAprilTagArea = idealAprilTagArea;


        addRequirements(limelight);
    }


    public Command getAutonomousCommand() {
        return this.selfAlignToAprilTag();
    }

    
    public Command selfAlignToAprilTag() {
        return new Command() {
            @Override
            public void initialize() {
            }
    
            @Override
            public void execute() {
                // Get the current area of the April tag
                double currentArea = limelight.getTargetArea();
    
                // Calculate the error between the current and ideal area
                double areaError = idealAprilTagArea - currentArea;
    
                // Use a simple proportional controller to calculate the speed
                // The kp value will need to be tuned for your specific robot
                double kp = 0.1;
                double speed = kp * areaError;
    
                // Get the horizontal difference of the April tag from the center of the camera view
                double horizontalDiff = limelight.getHorizontalDiff();
    
                // If the April tag is to the right of the center, turn right
                // If the April tag is to the left of the center, turn left
                // The kh value will need to be tuned for your specific robot
                double kh = 0.1;
                double turn = kh * horizontalDiff;
    
                // Drive the robot
                m_robotDrive.drive(speed, turn, 0, false, false);
            }
    
            @Override
            public void end(boolean interrupted) {
                // Stop the robot when the command ends
                m_robotDrive.drive(0, 0, 0, false, false);
            }
    
            @Override
            public boolean isFinished() {
                // This command never ends on its own, but it can be interrupted by other commands
                return false;
            }
    
            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(m_robotDrive, limelight);
            }
        };
    }

    





    
}
