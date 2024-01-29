package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveLib.CANSparkMaxUtil;
import frc.robot.subsystems.SwerveLib.OnboardModuleState;
import frc.robot.subsystems.SwerveLib.SwerveModuleConstants;
import frc.robot.subsystems.SwerveLib.CANSparkMaxUtil.Usage;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d desiredAngle;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final PIDController regController = 
    new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

  private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  private final PIDController turnFeedback =
    new PIDController(0.1, 0.0, 0.0, 0.02);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.desiredAngle = moduleConstants.desiredAngle;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /* Desired state for each swerve module. takes in speed and angle. If its openLoop, that means it is in teleop */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /* Reset wheel orientation to forward */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - desiredAngle.getDegrees();
    //integratedAngleEncoder.setPosition(integratedAngleEncoder.getPosition() - absolutePosition);
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  /* Settings for Angle Encoder */
  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig, 0.1);
  }

  /* Settings for Angle Motor */
  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setCANTimeout(5000);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  /* Settings for Drive Motor */
  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    driveMotor.setCANTimeout(5000);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  /* Gets the current position of the swerve module. This is an estimate */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
  }

  /* Sets the speed of the swerve module. If it's openLoop, then it takes in a percentage, otherwise, it calculates and runs a PID */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /* Sets the angle of the swerve module. */
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    
    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;

    // double turnAngleError = Math.abs(angle.getDegrees() - integratedAngleEncoder.getPosition());

    // double pidOut = regController.calculate(integratedAngleEncoder.getPosition(), angle.getDegrees());
    // // if robot is not moving, stop the turn motor oscillating
    // if (turnAngleError < Constants.Swerve.stickDeadband
    //     && Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
    //   pidOut = 0;

    // angleMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());

    // angleMotor.set(pidOut);
  }

  
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return (2 * Math.PI) / 60 * driveEncoder.getVelocity();
  }

  public void runCharacterization(double volts) {
    setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
    setDriveVoltage(volts);
  }

  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }
}