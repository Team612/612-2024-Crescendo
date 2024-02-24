
package frc.robot.commands.TrajectoryCommands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {
    private PathConstraints constraints = new PathConstraints(Constants.SwerveConstants.maxSpeed,
     Constants.SwerveConstants.maxAcceleration,
      Constants.SwerveConstants.maxAngularVelocity,
       Constants.SwerveConstants.maxAngularAcceleration);

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
            constraints,
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
            constraints,
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
            constraints,
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
            constraints,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath alignSpeaker(PoseEstimator estimation, Vision vision) {
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        double tagX = 0;
        double tagY = 0;
        Rotation2d tagAngle = new Rotation2d();
        double xChange = 0;
        double yChange = 0;
       
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            tagX = vision.return_tag_pose(7).getX();
            tagY = vision.return_tag_pose(7).getY();; 
            tagAngle = new Rotation2d(180);
            xChange = 1;
        } else {
            tagX = vision.return_tag_pose(4).getX();
            tagY = vision.return_tag_pose(4).getY();; 
            tagAngle = new Rotation2d(0);
            xChange = -1;
        }

        double offset = Constants.SwerveConstants.trackWidth / 2;

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + xChange, tagY + yChange, tagAngle)
            );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath alignAmp(PoseEstimator estimation, Vision vision) {
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        double tagX = 0;
        double tagY = 0;
        Rotation2d tagAngle = new Rotation2d();
        double xChange = 0;
        double yChange = 0;
       
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            tagX = vision.return_tag_pose(6).getX();
            tagY = vision.return_tag_pose(6).getY();; 
            tagAngle = new Rotation2d(90);
        } else {
            tagX = vision.return_tag_pose(5).getX();
            tagY = vision.return_tag_pose(5).getY();; 
            tagAngle = new Rotation2d(90);
        }

        double offset = Constants.SwerveConstants.trackWidth / 2;

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + xChange, tagY + yChange, tagAngle)
            );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
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
            tagAngle = new Rotation2d(-Units.degreesToRadians(180 - tagPose.getRotation().getDegrees()));
        }
        else{
            id = -1;
        }

        double offset = Constants.SwerveConstants.trackWidth / 2;
        
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
                constraints,
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
                constraints,
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
                constraints,
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
                constraints,
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
                constraints,
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
                constraints,
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
                constraints,
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
                constraints,
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
            //get the current RELATIVE notespace
            Transform2d notespace = vision.getNoteSpace();
            //double offset = Units.inchesToMeters(10); //center offset
            //add whatever translations to it
            notespace = new Transform2d(notespace.getX() - 0.5, notespace.getY(), notespace.getRotation());
            //transform the notespace to field relative coords. The angle is in estimatedPose, and the transformation is done by this angle.
            Pose2d transformedPose = estimatedPose.transformBy(notespace);
            //this is assuming that the current angle in the transformation is 0 degrees.
            // transformedPose.rotateBy(new Rotation2d(
            //     Units.degreesToRadians(-vision.getTargetYaw() + drivetrain.getNavxAngle().getDegrees()))
            //     );

            double endLocationX = transformedPose.getX();
            double endLocationY = (transformedPose.getY()); //- (2 * notespace.getY()) + 0.075
            System.out.println("---------------TRANSFORMATIONS----------------");
            System.out.println("End Location X: " + endLocationX);
            System.out.println("End Location Y: " + endLocationY);
            System.out.println("Transformed By (angle): " + transformedPose.getRotation().getDegrees());
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                // new Pose2d(x - 0.3,y,angle),
                // new Pose2d(12, y, angle)
                new Pose2d(endLocationX, endLocationY, angle)
            );

            PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
             constraints,
             new GoalEndState(0, angle));
             path.preventFlipping = true; //prevents the path from being flipped once the coords are reached
             return path;

        }
        System.out.println("NO TARGETS");
        return null;
    }

 

    
}