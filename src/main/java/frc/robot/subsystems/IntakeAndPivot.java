// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeAndPivot extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax m_Arm;
  private CANSparkMax m_IntakeMotor;
  public IntakeAndPivot() {
    m_Arm = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    m_Arm.setIdleMode(IdleMode.kBrake);
    m_Arm.getEncoder().setPositionConversionFactor(7/67); // converts into degrees
    m_IntakeMotor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    m_IntakeMotor.getEncoder().setPositionConversionFactor(7/67); // converts into degrees
  }
    /** Creates a new Intake. */
  public void setIntake(double speed) {
    m_IntakeMotor.set(speed);
  }
    public double getIntake() {
      return m_IntakeMotor.get();
    }
    public void resetEncoderPosIntake() {
      m_IntakeMotor.getEncoder().setPosition(0);
    }
  public void setArm(double speed) {
    m_Arm.set(speed);
  }
  public double getArm() {
    return m_Arm.get();
  }
  public void resetEncoderPosArm() {
    m_Arm.getEncoder().setPosition(0);
  }
  public void stop() {
    m_Arm.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}