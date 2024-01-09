// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToObject_Swerve extends CommandBase {
  private final ProfiledPIDController turnController = Constants.VisionConstants.rotationController;
  private int timer = 0;
  private double rotationspeed = 0;
  private final double offset = 6.0; // Adjust this offset as needed
  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;
  public double m_rotation;

  /** Creates a new TurnToObject_Swerve. */
  public TurnToObject_Swerve(Drivetrain drivetrain, Limelight lime) {
    m_drivetrain = drivetrain;
    m_limelight = lime;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setPipeline(1); // Set the pipeline
    m_drivetrain.drive(new Translation2d(0,0),0,false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.getTv()) {
          double angle = m_limelight.getTx() + offset; // Consider applying the offset
          timer = 0;
          //rotationspeed = -turnController.calculate(angle, 0);
          m_drivetrain.drive(new Translation2d(0, 0), angle, true);
      } else {
          timer++;
          rotationspeed = 0;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, false); // stops turning and driving
    //m_drivetrain.driveMecanum(0, 0, 0, 0);  <- need to switch to swerve
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_limelight.getTv()) {
      double yaw = m_limelight.getTx() + offset;
      return Math.abs(yaw) < 1; // Check within a small threshold
    }
    rotationspeed = 0;
    return timer / 20 > 10; // Check for a timer limit
  }
}
