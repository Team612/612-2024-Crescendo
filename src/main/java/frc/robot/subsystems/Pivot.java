// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private CANSparkMax m_IntakePivotMotor;
  private static Pivot pivotSubsystem = null;
  private static final double DEADZONE = 0.05;
  public Pivot() {
    m_IntakePivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
     m_IntakePivotMotor.setIdleMode(IdleMode.kBrake);
  }

   public boolean getIntakeLimitStateForward(){
    return m_IntakePivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
  }

  // return limit switch states
  public boolean getIntakeLimitStateReverse(){
    return m_IntakePivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
  }

    // move intake pivot
    public void rotateIntake(double rotate){
      if(Math.abs(rotate) < DEADZONE) rotate = 0;
      m_IntakePivotMotor.set(rotate);
    }

    public static Pivot getInstance(){
      if (pivotSubsystem == null){
        pivotSubsystem = new Pivot();
      }
      return pivotSubsystem;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
