// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax m_Arm;
  public Arm() {
    m_Arm = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_Arm.setIdleMode(IdleMode.kBrake);
    m_Arm.getEncoder().setPositionConversionFactor(360); // converts into degrees
  }
  public void set(double speed) {
    m_Arm.set(speed);
  }
  public double get() {
    return m_Arm.get();
  }
  public void resetEncoderPos() {
    m_Arm.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
