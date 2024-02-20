// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.MoveRollers;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  
  // Subsystems
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();

  // Commands
  private final IntakeUp m_intakeUp = new IntakeUp(m_intake);
  private final IntakeDown m_intakeDown = new IntakeDown(m_intake);
  private final MoveRollers m_moveRollers = new MoveRollers(m_intake);
  private final ShootNote m_shootNote = new ShootNote(m_shooter);

  public RobotContainer(){
    configureButtonBindings();
  }
  
  private void configureButtonBindings(){
    ControlMap.m_gunnerController.a().whileTrue(m_intakeUp);
    ControlMap.m_gunnerController.b().whileTrue(m_intakeDown);
    ControlMap.m_gunnerController.x().whileTrue(m_moveRollers);
    ControlMap.m_gunnerController.y().whileTrue(m_shootNote);
  }
  
  public Command getAutonomousCommand() {
    return m_moveRollers;
  }

}