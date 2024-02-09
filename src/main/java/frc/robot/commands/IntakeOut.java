// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndPivot;

public class IntakeOut extends Command {
  /** Creates a new IntakeCommand. */
  private boolean done=false;
  private final IntakeAndPivot m_Intake;
  public IntakeOut(IntakeAndPivot Intake) {
    m_Intake = Intake;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {} 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double timer = System.currentTimeMillis();
    m_Intake.setIntake(1);
    if (m_Intake.getIntake()!=1) {
      done=false;
    }else{
      m_Intake.setIntake(0);
      done=false;
    }
  }
  @Override
  public void end(boolean interrupted) {
    m_Intake.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}