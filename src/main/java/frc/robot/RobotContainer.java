// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.FieldOrientedDrive;
import frc.robot.commands.IntakeCommands.IntakeDown;
import frc.robot.commands.IntakeCommands.IntakeUp;
import frc.robot.commands.IntakeCommands.MoveRollers;
import frc.robot.commands.ShooterCommands.ShootNote;
import frc.robot.commands.TrajectoryCommands.FollowNote;
import frc.robot.commands.TrajectoryCommands.RunOnTheFly;
import frc.robot.commands.TrajectoryCommands.TrajectoryCreation;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.CharacterizationCommands.FeedForwardCharacterization;
import frc.robot.commands.CharacterizationCommands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  //Subsystem declerations
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final PoseEstimator m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
  private final TrajectoryConfiguration m_trajectoryConfig = TrajectoryConfiguration.getInstance();
  private final Vision m_vision = Vision.getVisionInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();

  // Autonomous commands
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final RunOnTheFly m_runOnTheFly = new RunOnTheFly(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);
  private final FollowNote m_moveToNote = new FollowNote(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);

  // Drive command
  private final DefaultDrive m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -ControlMap.m_driverController.getLeftY(),
            () -> -ControlMap.m_driverController.getLeftX(),
            () -> -ControlMap.m_driverController.getRightX());

  // Gunner commands
  private final IntakeDown m_intakeDown = new IntakeDown(m_intake);
  private final IntakeUp m_intakeUp = new IntakeUp(m_intake);
  private final MoveRollers m_moveRollers = new MoveRollers(m_intake);
  private final ShootNote m_shootNote = new ShootNote(m_shooter);

  //Drive subsystems declarations 
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private boolean isFieldOriented = true;

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Run on Fly", m_runOnTheFly);
    m_chooser.addOption("Test Path", m_trajectoryConfig.followPathGui("Double Path"));
    m_chooser.addOption("Move to Note", m_moveToNote);
    m_chooser.addOption("Swerve Characterization", new FeedForwardCharacterization(
              m_drivetrain,
              true,
              new FeedForwardCharacterizationData("drive"),
              m_drivetrain::runCharacterizationVolts,
              m_drivetrain::getCharacterizationVelocity));
    SmartDashboard.putData(m_chooser);
    // SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {
    // Driver button configs
    ControlMap.m_driverController.y().onTrue(new InstantCommand(() -> m_drivetrain.resetAlignment()));
    ControlMap.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    ControlMap.m_driverController.b().toggleOnTrue(m_defaultDrive);

    // Gunner button bindings
    ControlMap.m_gunnerController.a().whileTrue(m_intakeDown);
    ControlMap.m_gunnerController.b().whileTrue(m_intakeUp);
    ControlMap.m_gunnerController.x().whileTrue(m_moveRollers);
    ControlMap.m_gunnerController.y().whileTrue(m_shootNote);
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(
        new FieldOrientedDrive(
            m_drivetrain,
            () -> -ControlMap.m_driverController.getLeftY(),
            () -> -ControlMap.m_driverController.getLeftX(),
            () -> -ControlMap.m_driverController.getRightX()));
  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}