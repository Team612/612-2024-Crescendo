// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeIn extends CommandBase {
  /** Creates a new IntakeCommand. */
    private final Intake m_Intake;
  private final double m_timer;
  public IntakeIn(Intake Intake, double delay) {
    m_Intake = Intake;
    m_timer = delay;
    addRequirements(m_Intake);
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
    m_Intake.set(-1);
    if (timer - this.timer > m_timer) {
      m_Intake.set(0);
    }
  }
}
