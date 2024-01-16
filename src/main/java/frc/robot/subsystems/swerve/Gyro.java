// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;


import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gyro extends SubsystemBase {

  public ADIS16470_IMU gyro;
  public IMUAxis pitchAxis;
  public IMUAxis rollAxis;
  public IMUAxis yawAxis;

  public Rotation2d yawOffset = new Rotation2d(0);
  public Rotation2d pitchOffset = new Rotation2d(0);
  public Rotation2d rollOffset = new Rotation2d(0);

  /**
   * Creates a new Gyro, which is a wrapper for the Pigeon IMU and stores an offset so we don't
   * have to directly zero the gyro
   */
  public Gyro() {
    gyro = new ADIS16470_IMU();
    yawAxis = gyro.getYawAxis();

    gyro.setYawAxis(pitchAxis);
    pitchAxis = gyro.getYawAxis();

    gyro.setYawAxis(rollAxis);
    rollAxis = gyro.getYawAxis();

    gyro.setYawAxis(yawAxis);

    yawAxis = gyro.getYawAxis();
    
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
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double getRawYaw(){
    return gyro.getAngle();
  }

  public void resetYaw(){
    gyro.reset();
  }
  
  public double getYawRate(){
    return gyro.getRate();
  }



  public void setGyroPitchOffset(double degrees) {
    gyro.setYawAxis(pitchAxis);
    pitchOffset = getRawRot2dPitch().minus(Rotation2d.fromDegrees(degrees));
    gyro.setYawAxis(yawAxis);
  }

  public Rotation2d getProcessedRot2dPitch() {
    return getRawRot2dPitch().minus(pitchOffset).times(-1);
  }

  public Rotation2d getRawRot2dPitch() {
    gyro.setYawAxis(pitchAxis);
    double angle = gyro.getAngle();
    gyro.setYawAxis(yawAxis);
    return Rotation2d.fromDegrees(angle);
  }

  public double getRawPitch(){
    gyro.setYawAxis(pitchAxis);
    double angle = gyro.getAngle();
    gyro.setYawAxis(yawAxis);
    return angle;
  }

  public void resetPitch(){
    gyro.setYawAxis(pitchAxis);
    gyro.reset();
    gyro.setYawAxis(yawAxis);
  }

  public double getPitchRate(){
    gyro.setYawAxis(pitchAxis);
    double rate = gyro.getRate();
    gyro.setYawAxis(yawAxis);
    return rate;
  }





  public void setGyroRollOffset(double degrees) {
    gyro.setYawAxis(rollAxis);
    rollOffset = getRawRot2dRoll().minus(Rotation2d.fromDegrees(degrees));
    gyro.setYawAxis(yawAxis);
  }

  public Rotation2d getProcessedRot2dRoll() {
    return getRawRot2dRoll().minus(rollOffset).times(-1);
  }

  public Rotation2d getRawRot2dRoll() {
    gyro.setYawAxis(rollAxis);
    double angle = gyro.getAngle();
    gyro.setYawAxis(yawAxis);
    return Rotation2d.fromDegrees(angle);
  }

  public double getRawRoll(){
    gyro.setYawAxis(rollAxis);
    double angle = gyro.getAngle();
    gyro.setYawAxis(yawAxis);
    return angle;
  }

  public void resetRoll(){
    gyro.setYawAxis(rollAxis);
    gyro.reset();
    gyro.setYawAxis(yawAxis);
  }

  public double getRollRate(){
    gyro.setYawAxis(rollAxis);
    double rate = gyro.getRate();
    gyro.setYawAxis(yawAxis);
    return rate;
  }




  public void reset(){
    this.resetRoll();
    this.resetPitch();
    this.resetYaw();
  }
    
}
