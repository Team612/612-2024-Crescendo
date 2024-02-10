// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterOut extends Command {
  /** Creates a new LeftShooter. */
  private final Shooter m_Shooter;
  private boolean done = false;
  public ShooterOut(Shooter Shooter) {
    m_Shooter = Shooter;
    addRequirements(m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  double timer;
  @Override
  public void initialize() {
      m_Shooter.set(0); 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Shooter.set(1);
      if (m_Shooter.get() != 1) {
        done = false;
      } else  {
        done = true;
      }
  }
  // Called once thy command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.set(0);
  }

  // Returns true when the command should end.....?
  @Override
  public boolean isFinished() {
    return done;
  }
}