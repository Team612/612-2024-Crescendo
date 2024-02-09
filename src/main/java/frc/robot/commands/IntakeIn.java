// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndPivot;

public class IntakeIn extends Command{
  /** Creates a new IntakeCommand. */
  private final IntakeAndPivot m_IntakeAndPivot;
  private boolean done=false;
  public IntakeIn(IntakeAndPivot IntakeAndPivot) {
    m_IntakeAndPivot = IntakeAndPivot;
    addRequirements(m_IntakeAndPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {} 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeAndPivot.setIntake(-1);
    if (m_IntakeAndPivot.getIntake() != -1) {
      done = false;
    } else {
      done = true;
    }
  }
  @Override
  public void end(boolean isFinished) {
    m_IntakeAndPivot.setIntake(0);
  }
  @Override
  public boolean isFinished() {
    return done;
  }
}
