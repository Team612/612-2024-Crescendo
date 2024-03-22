// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootNoteSpeaker extends Command {
  private final Shooter m_Shooter;
  private Timer time = new Timer();
  /** Creates a new ShootNote. */
  public ShootNoteSpeaker(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.shoot(Constants.ShooterConstants.shooterLeftSpeedSpeaker, Constants.ShooterConstants.shooterRightSpeedSpeaker);
    m_Shooter.setTime(time.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.shoot(0, 0);
    time.stop();
    time.reset();
    m_Shooter.setTime(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}