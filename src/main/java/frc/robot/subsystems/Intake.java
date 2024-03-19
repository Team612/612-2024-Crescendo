// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import com.revrobotics.SparkLimitSwitch;

public class Intake extends SubsystemBase {
  private static final double DEADZONE = 0.05;
  private TalonSRX m_IntakeRollerMotor;
  private AnalogInput IRSensor = new AnalogInput(Constants.IntakeConstants.IRport);
  static Intake instance = null;
  /** Creates a new Intake. */
  public Intake() {
    m_IntakeRollerMotor = new TalonSRX(Constants.IntakeConstants.rollerID);
  }

  // create instance of intake
  public static Intake getInstance(){
    if(instance == null){
      instance = new Intake();
    }
    return instance;
  }
  
  // move intake rollers
  public void moveRollers(double rotate){
    if(Math.abs(rotate) < DEADZONE) rotate = 0;
    m_IntakeRollerMotor.set(TalonSRXControlMode.PercentOutput, rotate);
  }

  public double getIRSensor(){
    return IRSensor.getVoltage();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("limit forward", getIntakeLimitStateForward());
    // SmartDashboard.putBoolean("limit reverse", getIntakeLimitStateReverse());
    // SmartDashboard.putNumber("IR Sensor", getIRSensor());
    // This method will be called once per scheduler run
  }
}