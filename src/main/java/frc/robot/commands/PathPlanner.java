// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// package frc.robot.commands;

// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;

// // import com.path

// import edu.wpi.first.wpilibj2.command.Command;

// public class PathPlanner extends Command {
//   /** Creates a new PathPlanner. */
//   public PathPlanner() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
//         new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
//         new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
//         new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
//     );
//     PathPlannerPath path = new PathPlannerPath(
//       bezierPoints,
//       new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
//       new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
  

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
