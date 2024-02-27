// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class MoveRollersIn extends Command {
  private final Intake m_Intake;
  private double count = 0;
  /** Creates a new MoveRollers. */
  public MoveRollersIn(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Intake.getIRSensor() > 0.3) {
      count++;
    } else {
      count = 0;
    }
    m_Intake.moveRollers(-Constants.IntakeConstants.rollerSpeedIntake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_Intake.getIRSensor();
    return count >= 2;
  }
}