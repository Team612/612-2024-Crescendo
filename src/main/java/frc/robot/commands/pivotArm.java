// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class pivotArm extends CommandBase {
  /** Creates a new pivotArm. */
  private final Arm m_Arm;
  private final Boolean m_toShooter;
  private int m_turnAmount;
  private int amountTurned=0;
  public pivotArm(Arm Arm, Boolean toShooter, int turnAmount) {
    m_Arm = Arm;
    m_toShooter = toShooter;
    m_turnAmount = turnAmount;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_toShooter) {
      // going to the shooter
      if(m_turnAmount<amountTurned){
        m_Arm.set(-1);
        amountTurned--;
      }
      else{
        m_Arm.set(0);
      }

    } else {
      // going to the intake
      if(m_turnAmount>amountTurned){
        m_Arm.set(1);
        amountTurned++;
      }
      else{
        m_Arm.set(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
