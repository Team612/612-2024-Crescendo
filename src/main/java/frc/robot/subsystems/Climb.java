// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimb;
  private final CANSparkMax rightClimb;
  static Climb instance;
  /** Creates a new Climb. */
  public Climb() {
    leftClimb = new CANSparkMax(Constants.ClimbConstants.climbLeftID, MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.ClimbConstants.climbRightID, MotorType.kBrushless);
  }

  public static Climb getInstance(){
    if(instance == null){
      instance = new Climb();
    }
    return instance;
  }

  public void setSpeed(double speedLeft, double speedRight) {
    leftClimb.set(speedLeft);
    rightClimb.set(speedRight);
  }


  public void freezeMotors() {
    leftClimb.set(0);
    rightClimb.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
