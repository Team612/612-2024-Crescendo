package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private static AHRS navx;

    private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

    static Drivetrain drivetrain;

    public Drivetrain() {
        navx = new AHRS(I2C.Port.kMXP);
        navx.reset();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), // front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), // front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), // back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants)  // back right
        };
    }

    public static Drivetrain getInstance(){
      if (drivetrain == null){
         drivetrain = new Drivetrain();
      }
      return drivetrain;
    }

    public void autoDrive(ChassisSpeeds speeds) {
      SwerveModuleState[] swerveModuleStates =
          Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
      
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
  
      for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
      }
    }

    public ChassisSpeeds getChassisSpeeds() {
      ChassisSpeeds result = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
      return result;
    }

    public SwerveModuleState[] getStates() {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (SwerveModule mod : mSwerveMods) {
        states[mod.moduleNumber] = mod.getState();
      }
      return states;
    }
  
    public double getEncoderMeters() {
      double sum = 0.0;
      SwerveModulePosition[] positions = getPositions();
      for (SwerveModulePosition pos : positions) {
        sum += pos.distanceMeters;
      }
      return sum / 4.0;
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getNavxAngle()
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    
  public void driveRobotRelative(
    Translation2d translation, double rotation, boolean isOpenLoop) {
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
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
          states[i] = mSwerveMods[i].getState();
        }
        return states;
      }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
          positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
      }

    public ChassisSpeeds getSpeed() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
      navx.zeroYaw();
    }

    public Rotation2d getNavxAngle(){
      return Rotation2d.fromDegrees(-navx.getAngle());
    }
  
    public Rotation2d getYaw(){
      return Rotation2d.fromDegrees(navx.getYaw());
    }
    public Rotation2d getPitch(){
      return Rotation2d.fromDegrees(navx.getPitch());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void resetAlignment() {
      for(SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
      }
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

    @Override
    public void periodic(){;
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        if (isCharacterizing) {
          // Run in characterization mode
          for (SwerveModule mod : mSwerveMods) {
            mod.runCharacterization(characterizationVolts);
          }
        }
    }
}

/*     public class Limelight extends SubsystemBase {
        double tx = 0;
        double ty = 0;
        double tv = 0;
        double ta = 0;
        private SendableChooser<Boolean> m_limelightSwitch = new SendableChooser<>();

        public Limelight() {
            m_limelightSwitch.setDefaultOption("On", true);
            m_limelightSwitch.addOption("Off", false);

            SmartDashboard.putData("Limelight Switch", m_limelightSwitch);
        }

        //rotation for vision tracking-- we need to correct rotation on the limelight

        @Override
        public void periodic() {
            tx =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("tx")
                .getDouble(0);
            SmartDashboard.putNumber("tx", tx);

            ty =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("ty")
                .getDouble(0);
            SmartDashboard.putNumber("ty", ty);

            tv =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("tv")
                .getDouble(0);
            SmartDashboard.putBoolean("tv", tv >= 1.0);

            ta =
            NetworkTableInstance
                .getDefault()
                .getTable("limelight")
                .getEntry("ta")
                .getDouble(0);
            SmartDashboard.putBoolean("target valid", ta >= 1.0);

            SmartDashboard.putNumber("getSteeringValue", getSteeringValue());
        }

        public void setToAprilTags() {
            NetworkTableInstance
            .getDefault()
            .getTable("limelight")
            .getEntry("pipeline")
            .setNumber(0.0);
        }

        public void setToRetroreflectiveTape() {
            NetworkTableInstance
            .getDefault()
            .getTable("limelight")
            .getEntry("pipeline")
            .setNumber(1.0);
        }

        public double getSteeringValue() {
            double STEER_K = 0.1;

            double signumtx = Math.signum(tx);

            // if tv = 0, target is not valid so return 0.0
            if (tv == 0) {
            return 0.0;
            }

            if (m_limelightSwitch.getSelected() == false) {
            return 0.0;
            }

            double txAbs = Math.abs(tx);
            double txDeadband = txAbs - Constants.LIMELIGHT_DEADBAND;

            if (txDeadband < 0) {
            return 0.0;
            }

            double minDriveWithSine = signumtx * Constants.MIN_STEER_K;
            double steer_cmd = tx * STEER_K;
            double finalSteerCmd = minDriveWithSine + steer_cmd;

            return finalSteerCmd;
        }
    }
}

*/