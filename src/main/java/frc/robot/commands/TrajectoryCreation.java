
package frc.robot.commands;

import java.util.List;

import javax.tools.ForwardingJavaFileManager;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.ShuffleBoardButtons;
import frc.robot.subsystems.Drivetrain;
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

    // public PathPlannerPath moveToNote(PoseEstimator estimation, Vision vision) {
    //     Pose2d estimatedPose = estimation.getCurrentPose();
    //     double x = estimatedPose.getX();
    //     double y = estimatedPose.getY();
    //     Rotation2d angle = estimatedPose.getRotation();

    //     vision.getCamera().setPipelineIndex(1);

    //     PhotonPipelineResult result = vision.getCamera().getLatestResult();
    //     double CAMERA_HEIGHT_METERS = Units.inchesToMeters(18.5);
    //     double TARGET_HEIGHT_METERS = 0;
    //     double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    //     double GOAL_RANGE_METERS = Units.feetToMeters(1);
    //     if (result.hasTargets()) {
    //             // First calculate range
    //             double range = PhotonUtils.calculateDistanceToTargetMeters(
    //                             CAMERA_HEIGHT_METERS,
    //                             TARGET_HEIGHT_METERS,
    //                             CAMERA_PITCH_RADIANS,
    //                             Units.degreesToRadians(result.getBestTarget().getPitch()));
                                
    //             double absoluteYaw = angle.getDegrees() + result.getBestTarget().getYaw();
    //             List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //                 new Pose2d(x, y, angle),
    //                 new Pose2d((range - GOAL_RANGE_METERS) * Math.cos(absoluteYaw), (range - GOAL_RANGE_METERS) * Math.sin(absoluteYaw), new Rotation2d(absoluteYaw))
    //             );

    //             // Create the path using the bezier points created above
    //             PathPlannerPath path = new PathPlannerPath(
    //                 bezierPoints,
    //                 new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    //                 new GoalEndState(0.0, new Rotation2d(absoluteYaw)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    //             );
    //             vision.getCamera().setPipelineIndex(0);
    //             // Prevent the path from being flipped if the coordinates are already correct
    //             path.preventFlipping = true;
    //             return path;
    //         }
    //         List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //             new Pose2d(x, y, angle)
    //         );
    //         vision.getCamera().setPipelineIndex(0);
    //         // Create the path using the bezier points created above
    //         PathPlannerPath path = new PathPlannerPath(
    //             bezierPoints,
    //             new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    //             new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    //         );

    //         // Prevent the path from being flipped if the coordinates are already correct
    //         path.preventFlipping = true;
    //         return path;
    // }

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
            tagAngle = new Rotation2d(-Units.degreesToRadians(180 - tagPose.getRotation().getDegrees()));
        }
        else{
            id = -1;
        }

        double offset = Constants.Swerve.trackWidth / 2;
        
        if(id == 1 || id == 2 || id == 15) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5, tagY + 0.866 + offset, tagAngle)
            );
            System.out.println(tagX);
            System.out.println(tagY);
            System.out.println(tagAngle);

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 3 || id == 4 || id == 13) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 1, tagY + offset, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 5 || id == 6) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - offset, tagY - 1, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 7 || id == 8 || id == 14) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 1, tagY - offset, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 9 || id == 10 || id == 12) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 0.5, tagY + 0.866, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 11) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 0.5, tagY - 0.866, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 16) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5 - 0.433, tagY - 0.866 + 0.25, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
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

    public PathPlannerPath noteOnTheFly(PoseEstimator estimation, Vision vision, Drivetrain drivetrain){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
        boolean hasTargets = vision.hasTarget();

        if (hasTargets){
            Transform2d notespace = vision.getNoteSpace();
            double offset = Units.inchesToMeters(10); //center offset
            notespace = new Transform2d(notespace.getX() - 0.5, notespace.getY(), notespace.getRotation());
            Pose2d transformedPose = estimatedPose.transformBy(notespace);
            double endLocationX = transformedPose.getX();
            double endLocationY = transformedPose.getY() - (2 * notespace.getY()) + 0.075;
            Rotation2d endLocationRotation = transformedPose.getRotation();
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                // new Pose2d(x - 0.3,y,angle),
                // new Pose2d(12, y, angle)
                new Pose2d(endLocationX, endLocationY, angle)
            );

            PathPlannerPath path = new PathPlannerPath(bezierPoints,
             new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), 
             new GoalEndState(0, angle));
             path.preventFlipping = true; //prevents the path from being flipped once the coords are reached
             return path;

        }
        System.out.println("NO TARGETS");
        return null;
    }
}