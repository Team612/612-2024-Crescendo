// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  // Intake Constants
  public static class IntakeConstants{
    public static final int pivotID = 9;
    public static final int rollerID = 10;
    public static final double intakeUpSpeed = 0.1;
    public static final double intakeDownSpeed = -0.1;
    public static final double rollerSpeed = 0.1;
  }

  // Shooter Constants
  public static class ShooterConstants{
    public static final int shooterLeftID = 11;
    public static final int shooterRightID = 12;
    public static final double shooterLeftSpeed = 0.1;
    public static final double shooterRightSpeed = 0.1;
  }
}