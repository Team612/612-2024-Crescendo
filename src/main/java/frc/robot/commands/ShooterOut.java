// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterOut extends CommandBase {
  /** Creates a new LeftShooter. */
  private final Shooter m_Shooter;
  private final double m_timer;
  private final boolean m_full;
  public ShooterOut(Shooter Shooter, double delay, boolean full) {
    m_Shooter = Shooter;
    m_timer = delay;
    m_full = full;
    addRequirements(m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  double timer;
  @Override
  public void initialize() {
    timer = System.currentTimeMillis();
  } 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timer = System.currentTimeMillis();
    if (m_full) {
      m_Shooter.set(1);
      if (timer - this.timer > m_timer) {
        m_Shooter.set(0);
      }
    } else {
      m_Shooter.set(0.5);
      if (timer - this.timer > m_timer*2) {
        m_Shooter.set(0);
      }
    }
  }

  // Called once thy command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.....?
  @Override
  public boolean isFinished() {
    return false;
  }
}
