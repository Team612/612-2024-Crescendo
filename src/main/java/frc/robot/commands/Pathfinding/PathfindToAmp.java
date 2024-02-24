// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class PathfindToAmp extends Command {
  private Drivetrain driveSystem;
  private Vision m_vision;
  private PoseEstimator poseEstimatorSystem;
  private Pose2d targetPose;

  private Command controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public PathfindToAmp(Drivetrain d, PoseEstimator p, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSystem = d;
    poseEstimatorSystem = p;
    m_vision = v;
    targetPose = new Pose2d();
    addRequirements(d, v, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      targetPose = new Pose2d(
        m_vision.return_tag_pose(6).getX(),
        m_vision.return_tag_pose(6).getY() - 1,
        new Rotation2d(Units.degreesToRadians(90))
      );
    } else {
      targetPose = new Pose2d(
        m_vision.return_tag_pose(5).getX(),
        m_vision.return_tag_pose(5).getY() - 1,
        new Rotation2d(Units.degreesToRadians(90))
      );
    }
    controllerCommand = AutoBuilder.pathfindToPose(
              targetPose,
              new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration),
              0.0, // Goal end velocity in meters/sec
              0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}