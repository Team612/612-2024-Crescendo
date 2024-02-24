
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndPivot;

public class PivotArm extends Command {
  /** Creates a new pivotArm. */
  private final IntakeAndPivot m_Arm;
  public Boolean done = false; //to check if program is done
  private final boolean m_toShooter;
  private int m_turnAmount;
  private int amountTurned=0;
  public PivotArm(IntakeAndPivot Arm, Boolean toShooter, int turnAmount) {
    m_Arm = Arm;
    m_toShooter = toShooter;
    m_turnAmount = turnAmount;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_Arm.setArm(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_toShooter) {

      // going to the shooter position
      if(m_turnAmount<amountTurned){
        m_Arm.setArm(-1);
        amountTurned--;
      }
      else{
        m_Arm.setArm(0);
      }

    } else { //not going to shooter
      // going to the intake
      if(m_turnAmount>amountTurned){
        m_Arm.setArm(1);
        amountTurned++;
      }else{
        m_Arm.setArm(0);
      }
    }
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
