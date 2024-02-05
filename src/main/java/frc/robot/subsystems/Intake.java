// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax m_IntakeMotor;
  public Intake() {
    m_IntakeMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    m_IntakeMotor.getEncoder().setPositionConversionFactor(360); // converts into degrees
  }
  public void set(double speed) {
    m_IntakeMotor.set(speed);
  }
  public double get() {
    return m_IntakeMotor.get();
  }
  public void resetEncoderPos() {
    m_IntakeMotor.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
