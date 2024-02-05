// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShooterOut;
import frc.robot.commands.pivotArm;
import frc.robot.commands.ShooterIn;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.controls.ControlMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_IntakeSubsystem = new Intake();
  private final Arm m_arm = new Arm();
  private final Shooter m_Shooter = new Shooter();

//commands 
  private final IntakeIn m_IntakeIn = new IntakeIn(m_IntakeSubsystem, 2);
  private final IntakeOut m_IntakeOut = new IntakeOut(m_IntakeSubsystem, 2);
  private final ShooterOut m_ShooterOut = new ShooterOut(m_Shooter, 2, true);
  private final ShooterIn m_ShooterIn= new ShooterIn(m_Shooter, 2, true);
  private final ShooterOut m_ShooterOutHalf = new ShooterOut(m_Shooter, 2, false);
  private final ShooterIn m_ShooterInHalf = new ShooterIn(m_Shooter, 2, false);
  private final pivotArm m_PivotArmToShooter = new pivotArm(m_arm, true, -1);
  private final pivotArm m_PivotArmToIntake = new pivotArm(m_arm, false, 1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    ControlMap.GUNNER_RB.whenPressed(m_ShooterIn);
    ControlMap.GUNNER_LB.whenPressed(m_ShooterOut);
    ControlMap.blue1.whenPressed(m_IntakeIn);
    ControlMap.blue2.whenPressed(m_IntakeOut);
    ControlMap.red4.whenPressed(m_PivotArmToShooter);
    ControlMap.red5.whenPressed(m_PivotArmToIntake);
    ControlMap.green1.whenPressed(m_ShooterInHalf);
    ControlMap.green2.whenPressed(m_ShooterOutHalf);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_IntakeIn;
  }
}
