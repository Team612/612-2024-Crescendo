
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

    public TrajectoryConfig config = new TrajectoryConfig(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration)
        .setKinematics(Constants.Swerve.swerveKinematics);

    public TrajectoryConfig config_backwards = new TrajectoryConfig(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
    

    public Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2,0), new Translation2d(2,-2), new Translation2d(0, -2)),
        new Pose2d(0,0, new Rotation2d(0)), 
        config); 
    
    public Trajectory StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y-0.5)),
            new Pose2d(x,y-1, new Rotation2d(Units.degreesToRadians(degrees))),
            config_backwards
        );
    }

    public Trajectory StrafeLeftMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y+0.5)),
            new Pose2d(x,y+1, new Rotation2d(Units.degreesToRadians(degrees))),
            config
        );
    }

    public Trajectory ForwardMeter(PoseEstimator estimation){

        //figure out a way to find an initial position 

        // System.out.println("*************************** PRINT **********************************");
        // var currentPose = estimation.getCurrentPose();
        // System.out.println(currentPose);
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        

        // System.out.println("*************************** END PRINT **********************************");

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x + 0.5,y)),
            new Pose2d(x + 1.0, y, new Rotation2d(degrees)),
            config
        );
    }

    public Trajectory BackwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x-2.5,y)),
            new Pose2d(x-4.75, y, new Rotation2d(degrees
            )),
            config_backwards
        );
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

        if(result.hasTargets()){
            id = vision.getCamera().getLatestResult().getBestTarget().getFiducialId();


            Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
            tagX = tagPose.getX();
            tagY = tagPose.getY();
        }
        else{
            id = -1;
        }

        double offset = Units.inchesToMeters(5);
        System.out.println("current" + x + ' ' + y);
        System.out.println("tag" + tagX + ' ' + tagY);
        
        //if(id == 3 || id == 4) {
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, Rotation2d.fromDegrees(0))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAcceleration, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        //}
    }
}