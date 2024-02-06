// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Proxy;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.Characterization.FeedForwardCharacterization;
import frc.robot.commands.Characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ShooterOut;
import frc.robot.commands.ShooterIn;
import frc.robot.commands.PivotArm;
import frc.robot.commands.RunOnTheFly;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.commands.TrajectoryCreation;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ClimbArm;
import frc.robot.commands.RetractArm;


 /* This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_IntakeSubsystem = new Intake();
  private final Arm m_arm = new Arm();
  private final Shooter m_Shooter = new Shooter();
  private final PoseEstimator m_PoseEstimator = new PoseEstimator();
  private final Vision m_Vision = Vision.getVisionInstance();
  private final Climb m_Climb = new Climb();
  private final TrajectoryCreation m_Trajectory = new TrajectoryCreation();
//commands 
  private final IntakeIn m_IntakeIn = new IntakeIn(m_IntakeSubsystem, 2);
  private final IntakeOut m_IntakeOut = new IntakeOut(m_IntakeSubsystem, 2);
  private final ShooterOut m_ShooterOut = new ShooterOut(m_Shooter, 2, true);
  private final ShooterIn m_ShooterIn= new ShooterIn(m_Shooter, 2, true);
  private final ShooterOut m_ShooterOutHalf = new ShooterOut(m_Shooter, 2, false);
  private final ShooterIn m_ShooterInHalf = new ShooterIn(m_Shooter, 2, false);
  private final PivotArm m_PivotArmToShooter = new PivotArm(m_arm, true, -1);
  private final PivotArm m_PivotArmToIntake = new PivotArm(m_arm, false, 1);
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final Limelight m_limelight = Limelight.getInstance();
              
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton align =
      new JoystickButton(driver, XboxController.Button.kY.value);
  // private final JoystickButton robotCentric =
  //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  //Drive subsystems declarations 
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  private final SequentialCommandGroup m_AutoClimb = new SequentialCommandGroup(
    new RunOnTheFly(m_drivetrain, m_PoseEstimator, m_Trajectory, m_Vision, rotationAxis).andThen(new ClimbArm(m_Climb)).andThen(new RetractArm(m_Climb))
  );
  
  private final SequentialCommandGroup m_AutoTeleOpScoringRoutine = new SequentialCommandGroup(
    new RunOnTheFly(m_drivetrain, m_PoseEstimator, m_Trajectory, m_Vision, rotationAxis).andThen(new ShooterOut(m_Shooter, 3, true))
  );

  private final SequentialCommandGroup m_AutoTeleOpIntakeRoutine = new SequentialCommandGroup(
    new RunOnTheFly(m_drivetrain, m_PoseEstimator, m_Trajectory, m_Vision, rotationAxis).andThen(new ShooterIn(m_Shooter, 3, true))
  );

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
    align.onTrue(new InstantCommand(() -> m_drivetrain.resetAlignment()));
    ControlMap.GUNNER_RB.toggleOnTrue(m_ShooterIn);
    ControlMap.GUNNER_LB.toggleOnTrue(m_ShooterOut);
    ControlMap.blue1.toggleOnTrue(m_IntakeIn);
    ControlMap.blue2.toggleOnTrue(m_IntakeOut);
    ControlMap.red4.toggleOnTrue(m_PivotArmToShooter);
    ControlMap.red5.toggleOnTrue(m_PivotArmToIntake);
    ControlMap.green1.toggleOnTrue(m_ShooterInHalf);
    ControlMap.green2.toggleOnTrue(m_ShooterOutHalf);
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(
        new DefaultDrive(
            m_drivetrain,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));
  }

  public void TeleopHeading(){
    
  }
  
  public Command getAutonomousCommand() {
    double dist = (14 - 10) / (Math.tan(Units.degreesToRadians(m_limelight.getTy())) * Math.cos(Units.degreesToRadians(m_limelight.getTx())));
    double tx = m_limelight.getTx();
    double areaCameraIsCovering = m_limelight.getTa()/1; // Area Camera is covering 
    final double ksVolts = 0.22;
    final double kvVoltSecondsPerMeter = 1.98;
    final double kaVoltSecondsSquaredPerMeter = 0.2;
    final double kTrackwidthMeters = 0.69;
    final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    final double kMaxSpeedMetersPerSecond = 3;
    final double kMaxAccelerationMetersPerSecondSquared = 1;
    // Example value only - as above, this must be tuned for your drive!
    final double kPDriveVel = 8.5;
    final double kRamseteB = 2;
    final double kRamseteZeta = 0.7;

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            ksVolts,
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        10);
        
    TrajectoryConfig config =
    new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
        
        // Note x = dist * cos 0, y = dist * sin 0
    Trajectory driveToNote =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // STARTING POSITION AND STARTING X, WILL BE CALCULATED USING APRIL TAGS
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0+1, (1 + dist*Math.sin(tx))/2)),
        // End 3 meters straight ahead of where we started, facing forward
        // ENDING POSITION AND ENDING X, POSITION OF NOTE
        new Pose2d(1 + dist*Math.cos(tx), 1 + dist*Math.sin(tx), new Rotation2d(90)),
        // Pass config
        config);

    Trajectory driveAroundObject =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // STARTING POSITION AND STARTING X, WILL BE CALCULATED USING APRIL TAGS
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1+dist, 1 + (Math.cos(tx))/dist)),
        // End 3 meters straight ahead of where we started, facing forward
        // ENDING POSITION AND ENDING X, POSITION OF NOTE
        new Pose2d(1 + dist-Math.sin(3*tx-90)*Math.sin(tx)/dist, 1 + Math.cos(tx)/dist + Math.cos(3*tx-90)*Math.sin(tx)/dist, new Rotation2d(90)),
        // Pass config
        config);


    Trajectory AprilTagAlignment =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // STARTING POSITION AND STARTING X, WILL BE CALCULATED USING APRIL TAGS
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1+dist, 1 + (Math.cos(tx))/dist)),
        // End 3 meters straight ahead of where we started, facing forward
        // ENDING POSITION AND ENDING X, POSITION OF NOTE
        new Pose2d(1 + dist-Math.sin(3*tx-90)*Math.sin(tx)/dist, 1 + Math.cos(tx)/dist + Math.cos(3*tx-90)*Math.sin(tx)/dist, new Rotation2d(90)),
        // Pass config
        config);
        
    return m_chooser.getSelected();
  }}