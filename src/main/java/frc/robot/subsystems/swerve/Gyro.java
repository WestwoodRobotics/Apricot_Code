// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ADIS16470_IMU_Better;


public class Gyro extends SubsystemBase {

  public ADIS16470_IMU_Better gyro;

  public Rotation2d yawOffset = new Rotation2d(0);


  /**
   * Creates a new Gyro, which is a wrapper for the ADIS16740 IMU and stores an offset so we don't
   * have to directly zero the gyro
   */
  public Gyro() {
    gyro = new ADIS16470_IMU_Better();
    zeroGyro();
  }

  /**
   * Zero the gyro
   */
  public void zeroGyro() {
    setGyroYawOffset(0);
  }

  public void setGyroYawOffset(double degrees) {
    yawOffset = getRawRot2dYaw().minus(Rotation2d.fromDegrees(degrees));
  }

  public Rotation2d getProcessedRot2dYaw() {
    return getRawRot2dYaw().minus(yawOffset).times(-1);
  }

  public Rotation2d getRawRot2dYaw() {
    return Rotation2d.fromDegrees(gyro.getXAngle());
  }

  public double getXAngle(){
    return gyro.getXAngle();
  }

  public double getYAngle(){
    return gyro.getYAngle();
  }

  public double getZAngle(){
    return gyro.getZAngle();
  }

  public void resetYaw(){
    gyro.reset();
  }
  
  public double getZRate(){
    return gyro.getZAngularRate();
  }

  public void reset(){
    this.resetYaw();
  }
    
}
