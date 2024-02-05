// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.Robot;
public class Climb extends SubsystemBase {
  private Robot m_robot;
  private final DoubleSolenoid m_piston1 = new DoubleSolenoid(Constants.PCM_2, Constants.solenoidType, Constants.firstSolenoid[1], Constants.firstSolenoid[0]);
  private final DoubleSolenoid m_piston2 = new DoubleSolenoid(Constants.PCM_2, Constants.solenoidType, Constants.secondSolenoid[1] ,Constants.secondSolenoid[0]);
  //Pushes the piston out
  /** Creates a new Climb. */
  public Climb() {}
  public void extendArm(){
    m_piston1.set(Value.kForward);
    m_piston2.set(Value.kForward);
    System.out.println("extend");
    m_robot.extendArm();
    Value Value = edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
    if (m_piston1.get().equals(Value)) {
      m_piston2.set(Value);
    }
    }
//Brings the piston in
public void retractArm(){
    m_piston1.set(Value.kReverse);
    m_piston2.set(Value.kReverse);
    System.out.println("retract");
    m_robot.retractArm();
    Value Value = edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
    if (m_piston1.get().equals(Value)) {
      m_piston2.set(Value);
    }
}
public void climb() {
  extendArm();
  retractArm();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
