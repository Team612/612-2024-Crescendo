// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
// import org.firstinspires.ftc.robotcore.external;
// import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import frc.robot.commands.Characterization.FeedForwardCharacterization;
import frc.robot.commands.Characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.IntakeAndPivot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ShooterOut;
import frc.robot.commands.PivotArm;
import frc.robot.commands.RunOnTheFly;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.commands.TrajectoryCreation;
import frc.robot.commands.autoClimb;
import frc.robot.commands.driveToClimb;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ClimbArm;
import frc.robot.commands.RetractArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Telemetry
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

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
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
