// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.MoveRollers;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystem declerations
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();

  // Command initialization
  private final IntakeDown m_intakeDown = new IntakeDown(m_intake);
  private final IntakeUp m_intakeUp = new IntakeUp(m_intake);
  private final MoveRollers m_moveRollers = new MoveRollers(m_intake);
  private final ShootNote m_shootNote = new ShootNote(m_shooter);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    ControlMap.m_gunnerController.a().whileTrue(m_intakeDown);
    ControlMap.m_gunnerController.b().whileTrue(m_intakeUp);
    ControlMap.m_gunnerController.x().whileTrue(m_moveRollers);
    ControlMap.m_gunnerController.y().whileTrue(m_shootNote);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_moveRollers;
  }
}
