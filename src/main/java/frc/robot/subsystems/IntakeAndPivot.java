// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.ejml.equation.ManagerFunctions.Input1;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeAndPivot extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder encPiv;
  private DigitalInput dig = new DigitalInput(0);
  private RelativeEncoder encIntake;

  public IntakeAndPivot() {
    m_pivotMotor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    m_IntakeMotor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    encPiv = m_pivotMotor.getEncoder();
    encIntake = m_IntakeMotor.getEncoder();
  }

  public void configurePivotMotor(){
    m_pivotMotor.setIdleMode(IdleMode.kBrake);
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    encIntake.setPositionConversionFactor(7/67);
    encPiv.setPositionConversionFactor(7/67); // converts into degrees
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
    m_pivotMotor.set(speed);
  }
  
  public double getArm() {
    return m_pivotMotor.getEncoder().getPosition();
  }
  public void resetEncoderPosArm() {
    m_pivotMotor.getEncoder().setPosition(0);
  }

  public boolean irSensorDetected(){
    if(dig.get()==true){
       setIntake(0);
       return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}