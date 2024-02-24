// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class JustTurn extends Command {
  /** Creates a new JustTurn. */
  private Drivetrain m_drivetrain;
  private Vision m_vision;

  private double initialPos = 0;
  private double initialAngle = 0;
  private double goalPos = 0;
  private double goalAngle = 0;

  private boolean doneTurning = false;
  public JustTurn(Drivetrain d, Vision v) {
    m_drivetrain = d;
    m_vision = v;
    addRequirements(d, v);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getNavxAngle().getDegrees();
    if(m_vision.hasTarget()) {
      goalPos = m_vision.getNoteRange() - 1;
      goalAngle = -m_vision.getTargetYaw();
    }
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle)) <= 0.1) {
      doneTurning = true;
    }
    if(doneTurning) {
      double curPos = goalPos - (Math.abs(m_drivetrain.getEncoderMeters() - initialPos));
      m_drivetrain.driveRobotRelative(new Translation2d(curPos, 0), 0, false);
    } else {
      double curAngle = goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle);
      m_drivetrain.driveRobotRelative(new Translation2d(), Units.degreesToRadians(curAngle) * 2, false);
      initialPos = m_drivetrain.getEncoderMeters();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}