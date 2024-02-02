// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.RunOnTheFly;
import frc.robot.commands.TrajectoryCreation;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final PoseEstimator m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
  private final TrajectoryConfiguration m_trajectoryConfig = TrajectoryConfiguration.getInstance();
  private final Vision m_vision = Vision.getVisionInstance();

  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final RunOnTheFly m_runOnTheFly = new RunOnTheFly(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);
  private final FollowTrajectory m_forward = new FollowTrajectory(m_drivetrain, m_poseEstimator, m_traj, 0);
              
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final DefaultDrive m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis));

  /* Driver Buttons */
  private final JoystickButton align =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton fieldCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton robotToField =
      new JoystickButton(driver, XboxController.Button.kB.value);

  //Drive subsystems declarations 
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private boolean isFieldOriented = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  // private final SequentialCommandGroup m_RedTopScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_RedBottomScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_BlueTopScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_BlueBottomScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  private void configureShuffleBoardBindings(){
    // m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    // m_chooser.addOption("Red Top Leave And Dock", new ProxyCommand(() -> m_RedTopLeaveAndDock));
    // m_chooser.addOption("Blue Top Leave And Dock", new ProxyCommand(() -> m_BlueTopLeaveAndDock));
    // m_chooser.addOption("Red Bottom Leave And Dock", new ProxyCommand(() -> m_RedBottomLeaveAndDock));
    // m_chooser.addOption("Blue Bottom Leave and Dock", new ProxyCommand(() -> m_BlueBottomLeaveAndDock));
    m_chooser.addOption("Run on Fly", m_runOnTheFly);
    m_chooser.addOption("Test Path", m_trajectoryConfig.followPathGui("First Path"));
    m_chooser.addOption("Forward one", m_forward);
    SmartDashboard.putData(m_chooser);
    // SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {
    align.onTrue(new InstantCommand(() -> m_drivetrain.resetAlignment()));
    fieldCentric.onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    robotToField.toggleOnTrue(m_defaultDrive);
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(
        new FieldOrientedDrive(
            m_drivetrain,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));
  }

  // public void TeleopHeading(){
  //   Rotation2d finalHeading = new Rotation2d(Units.degreesToRadians(-180));
  //   Rotation2d currentHeading = m_poseEstimator.getCurrentPose().getRotation();
  //   Rotation2d deltaHeading = finalHeading.minus(currentHeading);
  //   if(Robot.initAllianceColor == Alliance.Blue){
  //     m_drivetrain.setNavxAngleOffset(deltaHeading.plus(new Rotation2d(Units.degreesToRadians(0))));
  //   }
    
  //   if(Robot.initAllianceColor == Alliance.Red){
  //     m_drivetrain.setNavxAngleOffset(deltaHeading.plus(new Rotation2d(Units.degreesToRadians(180))));
  //   }
  // }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}