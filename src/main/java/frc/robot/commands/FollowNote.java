// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.crypto.dsig.Transform;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class FollowNote extends Command {
  private final Drivetrain driveSubsystem;
  private final PoseEstimator poseSubsystem;
  private final TrajectoryCreation m_traj;
  private final Vision visionSubsystem;
  private final double translation;
  private Transform2d notespace;

  private Command controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public FollowNote(Drivetrain d, PoseEstimator p, TrajectoryCreation traj, Vision v,
                    double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = d;
    poseSubsystem = p;
    m_traj = traj;
    visionSubsystem = v;
    translation = y;

    addRequirements(d, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = m_traj.noteOnTheFly(poseSubsystem, visionSubsystem,driveSubsystem);
    if (path == null) {
      System.out.println("NO TARGETS");
      end(true);
    }
    else {
    controllerCommand = AutoBuilder.followPath(path);
    controllerCommand.initialize();
    }
    //we dont want to update using apriltag during the trajectory
    
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
    System.out.println(" Current Pose: " + poseSubsystem.getCurrentPose().getY() + " Speed, " + driveSubsystem.getStates()[1].speedMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("--------------------DONE------------------");
    controllerCommand.end(interrupted);
    System.out.println(poseSubsystem.getCurrentPose().getX());
    System.out.println(poseSubsystem.getCurrentPose().getY());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}