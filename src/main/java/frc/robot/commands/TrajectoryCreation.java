
package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.ShuffleBoardButtons;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {
    
    public PathPlannerPath StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y - 1, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath StrafeLeftMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y + 1, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath ForwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x + 1, y, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath BackwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x - 1, y, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath onthefly(PoseEstimator estimation, Vision vision, double y_translation){
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
       
        PhotonPipelineResult result = vision.getCamera().getLatestResult();
        int id;
        double tagX = 0;
        double tagY = 0; 
        Rotation2d tagAngle = new Rotation2d();

        if(result.hasTargets()){
            id = vision.getCamera().getLatestResult().getBestTarget().getFiducialId();


            Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
            tagX = tagPose.getX();
            tagY = tagPose.getY();
            tagAngle = tagPose.getRotation();
            System.out.println(tagAngle.getDegrees());
        }
        else{
            id = -1;
        }

        double offset = Constants.Swerve.trackWidth / 2;
        System.out.println("current" + x + ' ' + y);
        System.out.println("tag" + tagX + ' ' + tagY);
        
        if(id == 1 || id == 2 || id == 15) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5, tagY + 0.866 + offset, new Rotation2d(-tagAngle.getDegrees()))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, new Rotation2d(-tagAngle.getDegrees())) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 3 || id == 4 || id == 13) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 1, tagY + offset, Rotation2d.fromDegrees(0))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 5 || id == 6) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY - 1 + offset, Rotation2d.fromDegrees(90))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 7 || id == 8 || id == 14) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, Rotation2d.fromDegrees(180))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 9 || id == 10 || id == 12) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, Rotation2d.fromDegrees(240))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(240)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 11) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, Rotation2d.fromDegrees(120))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(120)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 16) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, Rotation2d.fromDegrees(60))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, angle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        }
    }
}