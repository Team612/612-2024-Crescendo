// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TestMeter extends Command {
  /** Creates a new MoveToNote. */
  private Drivetrain m_drivetrain;

  private double initialPos = 0;
  public TestMeter(Drivetrain d) {
    m_drivetrain = d;
    addRequirements(d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPos = m_drivetrain.getEncoderMeters();
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);
  }

  // Called every time the scheduler runs while the command is scheddled.
  @Override
  public void execute() {
    double curPos = 1 - (Math.abs(m_drivetrain.getEncoderMeters() - initialPos));
    System.out.println(curPos);
    m_drivetrain.driveRobotRelative(new Translation2d(curPos, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
