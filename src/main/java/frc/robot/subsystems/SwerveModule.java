package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.*;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.subsystems.SwerveLib.CANSparkMaxUtil;
import frc.robot.subsystems.SwerveLib.OnboardModuleState;
import frc.robot.subsystems.SwerveLib.SwerveModuleConstants;
// import frc.robot.subsystems.SwerveLib.CANSparkMaxUtil.Usage;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class SwerveModule {
 public int moduleNumber;
 private Rotation2d lastAngle;
 private Rotation2d desiredAngle;

 private TalonFX angleMotor = new TalonFX(0, "612Test"); // Both of these construct a TalonFX on the CANivore bus named "612Test"
 private TalonFX driveMotor = new TalonFX(0, "612Test");
  private RelativeEncoder driveEncoder;
 private CANcoder angleEncoder;

 private CurrentLimitsConfigs currentLimitConfigurator = new CurrentLimitsConfigs();
 private OpenLoopRampsConfigs openLoopRampsConfigurator = new OpenLoopRampsConfigs();


//  private final PIDController regController =
//    new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

//  private final SimpleMotorFeedforward feedforward =
//    new SimpleMotorFeedforward(
//          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

 private final PIDController turnFeedback =
   new PIDController(0.1, 0.0, 0.0, 0.02);
  public double getEncoderPosition(TalonFX motor) { // gets position in degrees
   return motor.getRotorPosition().getValue();
 }

 public void setEncoderPosition(TalonFX motor, double angle) { // waits 50 seconds before proceeding by default. angle is in degrees
   motor.setPosition(angle*Constants.Swerve.conversionFactorAngle);
 }

 public double getDriveVelocity(TalonFX motor) {
   return motor.getRotorVelocity().getValue();
 }

  public void setDriveVelocity(TalonFX motor, double speed) { // normal speed between -1.0 and 1.0
    motor.set(speed);
  }
  public void resetFactoryDefaults(TalonFX motor, double timeout) { // defeault of 50 ms time to take
    motor.feed();
    motor.clearStickyFault_RemoteSensorReset(timeout);
    motor.clearStickyFaults(timeout);
    currentLimitConfigurator.withSupplyCurrentLimitEnable(false);
    openLoopRampsConfigurator.withDutyCycleOpenLoopRampPeriod(0);
    openLoopRampsConfigurator.withTorqueOpenLoopRampPeriod(0);
    openLoopRampsConfigurator.withVoltageOpenLoopRampPeriod(0);
  }
  public void setCurrentLimit(TalonFX motor, double limit) {
    currentLimitConfigurator.withSupplyCurrentLimitEnable(true);
    currentLimitConfigurator.withSupplyCurrentLimit(limit);
    motor.getConfigurator().apply(currentLimitConfigurator);
  }
  public void setOpenLoopRampRate(TalonFX motor, double ramprate) {
    openLoopRampsConfigurator.withDutyCycleOpenLoopRampPeriod(ramprate);
    openLoopRampsConfigurator.withTorqueOpenLoopRampPeriod(ramprate);
    openLoopRampsConfigurator.withVoltageOpenLoopRampPeriod(ramprate);
    motor.getConfigurator().apply(openLoopRampsConfigurator);
  }
 public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
   this.moduleNumber = moduleNumber;
   this.desiredAngle = moduleConstants.desiredAngle;

   /* Angle Encoder Config */
   angleEncoder = new CANcoder(moduleConstants.cancoderID);
   configAngleEncoder();

   /* Angle Motor Config */
   angleMotor = new TalonFX(moduleConstants.angleMotorID);
   configAngleMotor();

   /* Drive Motor Config */
   driveMotor = new TalonFX(moduleConstants.driveMotorID);
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
   setEncoderPosition(angleMotor, absolutePosition);
 }

 /* Settings for Angle Encoder */
 private void configAngleEncoder() {
   angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig, 0.1);
 }

 /* Settings for Angle Motor */
 private void configAngleMotor() {
   resetFactoryDefaults(angleMotor, 0);
   setCurrentLimit(angleMotor, Constants.Swerve.angleContinuousCurrentLimit);
   angleMotor.setInverted(Constants.Swerve.angleInvert);
   angleMotor.setControl(new DutyCycleOut(0, false, true, false, false));
   resetToAbsolute();
 }
  /* Settings for Drive Motor */
 private void configDriveMotor() {
   resetFactoryDefaults(driveMotor, 0);
   setCurrentLimit(driveMotor, Constants.Swerve.driveContinuousCurrentLimit);
   driveMotor.setInverted(Constants.Swerve.driveInvert);
   driveMotor.setControl(new DutyCycleOut(0, false, true, false, false));
   setOpenLoopRampRate(driveMotor, 0.25);
   setEncoderPosition(driveMotor, 0);
 }

 /* Gets the current position of the swerve module. This is an estimate */
 public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getAngle());
 }

 /* Sets the speed of the swerve module. If it's openLoop, then it takes in a percentage, otherwise, it calculates and runs a PID */
 private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
   if (isOpenLoop) {
     double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
     driveMotor.set(percentOutput);
   } else {
    //  driveController.setReference(
    //      desiredState.speedMetersPerSecond,
    //       .kVelocity,
    //      0,
    //      feedforward.calculate(desiredState.speedMetersPerSecond));
   }
 }

 /* Sets the angle of the swerve module. */
 private void setAngle(SwerveModuleState desiredState) {
   // Prevent rotating module if speed is less then 1%. Prevents jittering.
   Rotation2d angle =
       (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
           ? lastAngle
           : desiredState.angle;
  
  //  angleMotor.setReference(angle.getDegrees(), ControlType.kPosition);
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
   return Rotation2d.fromDegrees(getEncoderPosition(angleMotor));
 }

 public Rotation2d getCanCoder() {
   return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
 }

 public SwerveModuleState getState() {
   return new SwerveModuleState(getDriveVelocity(driveMotor), getAngle());
 }

 public double getVoltage() {
   return driveMotor.getMotorVoltage().getValue();
 }

 /** Returns the drive velocity in m/sec. */
 public double getCharacterizationVelocity() {
   return driveEncoder.getVelocity();
 }

 public void runCharacterization(double volts) {
   driveMotor.setVoltage(volts);
 }
}