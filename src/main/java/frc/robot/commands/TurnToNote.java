// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToNote extends Command {
  /** Creates a new TurnToNote. */
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private double initialAngle;
  private double initialPos;
  private double turnAngle;
  private double noteRange;
  private boolean doneTurning;
  public TurnToNote(Drivetrain d, Vision v) {
    m_drivetrain = d;
    m_vision = v;
    initialAngle = 0;
    initialPos = 0;
    turnAngle = 0;
    noteRange = 0;
    doneTurning = false;
    addRequirements(d, v);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getEncoderMeters();
    turnAngle = -m_vision.getTargetYaw() * 0.005772830242988; //meter for every degree
    noteRange = m_vision.getNoteRange();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Math.abs(turnAngle));
    System.out.println(Math.abs(m_drivetrain.getEncoderMeters() - initialAngle));
    if(Math.abs(m_drivetrain.getEncoderMeters() - initialAngle) > Math.abs(turnAngle)) {
      doneTurning = true;
      initialPos = m_drivetrain.getEncoderMeters();
    }
    if(doneTurning) {
      m_drivetrain.driveRobotRelative(new Translation2d(1, 0), 0, false);
    } else {
      m_drivetrain.driveRobotRelative(new Translation2d(), turnAngle / 2, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneTurning && (m_drivetrain.getEncoderMeters() - initialPos + 0.5) > noteRange;
  }
}
