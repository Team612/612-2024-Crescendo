
package frc.robot.subsystems;


import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain drivetrain = null;

  private SwerveModule[] mSwerveMods;

  private static AHRS navx;
  private Rotation2d navxAngleOffset;

  private Field2d field;

  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

  public Drivetrain() {
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    
    navx = new AHRS(I2C.Port.kMXP); 
    navxAngleOffset = new Rotation2d();
    navx.reset();
    //navx.calibrate;

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public static Drivetrain getInstance(){
    if (drivetrain == null){
       drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

  public void drive(
      Translation2d translation, double rotation, boolean isOpenLoop) {
    // SwerveModuleState[] swerveModuleStates =
    //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //                 translation.getX(), translation.getY(), rotation, getNavxAngle()));
    
    SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public double getPositionsDist() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      return mod.getDistance();
    }
    return 0;
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  public Rotation2d getNavxAngle(){
    return Rotation2d.fromDegrees(navx.getAngle());
  }

  public double getNavxAngleO(){
    return navx.getAngle();
  }
    
  // setter for setting the navxAngleOffset
  public void setNavxAngleOffset(Rotation2d angle){
    navxAngleOffset = angle;
  }

  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(navx.getYaw());
  }

  public double returnYaw(){
    return navx.getYaw();
  }

  public Rotation2d getPitch(){
    return Rotation2d.fromDegrees(navx.getPitch());
  }

  public double getRoll(){
    return navx.getRoll();
  }


  public void runCharacterizationVolts(double volts) {
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : mSwerveMods) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public void resetAlignment() {
    for(SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    if (isCharacterizing) {
      // Run in characterization mode
      for (SwerveModule mod : mSwerveMods) {
        mod.runCharacterization(characterizationVolts);
      }
    }
  }
}