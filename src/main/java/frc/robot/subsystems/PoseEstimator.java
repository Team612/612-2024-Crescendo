// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryCommands.TrajectoryCreation;
import frc.robot.commands.TrajectoryCommands.FollowNote;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  SwerveDrivePoseEstimator m_DrivePoseEstimator;
  PhotonPoseEstimator m_PhotonPoseEstimatorFront;
  PhotonPoseEstimator m_PhotonPoseEstimatorBack;
  Vision m_vision;
  Drivetrain m_drivetrain;
  private Field2d m_field;
  private boolean updateWithAprilTags;

  
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;

  //Matrix Stds for state estimate
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);

  //Matrix Stds for vision estimates
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  static PoseEstimator estimator = null;
  
  public PoseEstimator() {
    m_drivetrain = Drivetrain.getInstance();
    m_vision = Vision.getVisionInstance();
    m_field = new Field2d();
    updateWithAprilTags = true;
    SmartDashboard.putData("Field", m_field);


    m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.swerveKinematics, 
      m_drivetrain.getNavxAngle(), 
      m_drivetrain.getPositions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    m_PhotonPoseEstimatorBack = m_vision.getVisionPose();
    m_PhotonPoseEstimatorFront = m_vision.getVisionPose2();
  }

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }

  public void isUsingAprilTag(boolean b){
    updateWithAprilTags = b;
  }

  public void updateEachPoseEstimator(PhotonPoseEstimator poseEstimator, int camID){      

    if(m_vision.getApriltagCamera(camID).getLatestResult().hasTargets()) {
     poseEstimator.update().ifPresent(estimatedRobotPose -> {
      System.out.println(1);
      var estimatedPose = estimatedRobotPose.estimatedPose;
      System.out.println(estimatedRobotPose.targetsUsed.size());

      // m_DrivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), FIELD_LENGTH_METERS);
     
      // Make sure we have a new measurement, and that it's on the field
      //current issue: once the robot sees an apriltag, hasTarget returns false??????
      // if(m_vision.getApriltagCamera(camID).getLatestResult().hasTargets()){
      if (m_vision.getApriltagCamera(camID).getLatestResult().getBestTarget().getFiducialId() >= 0){
        
      if (
        estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
      estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
      && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
        if (estimatedRobotPose.targetsUsed.size() >= 1) {
        
          for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
            Pose3d targetPose = m_vision.return_tag_pose(target.getFiducialId());
            Transform3d bestTarget = target.getBestCameraToTarget();
            Pose3d camPose;
            if (camID == 2){
            // Transform3d robotToCam = new Transform3d(bestTarget.getTranslation().plus(m_vision.getRobotToCam(camID).getTranslation()), new Rotation3d(0,0,0));
            // camPose = targetPose.transformBy(robotToCam.inverse()); 
              camPose = targetPose.transformBy(bestTarget.inverse().plus(m_vision.getRobotToCam(camID)));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
              if(camPose.getRotation().toRotation2d().getDegrees() < 180) {
                camPose.rotateBy( new Rotation3d(0, 0, 180));
                //camPose.plus(new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, 180)));
              } else {
                camPose.rotateBy( new Rotation3d(0, 0, -180));
                //camPose.plus(new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, -180)));
              }
            }           
            else {
              camPose = targetPose.transformBy(bestTarget.inverse().plus(m_vision.getRobotToCam(camID)));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
            }
            System.out.println(camPose.toPose2d());
      //       //checking from the camera to the tag is less than 4
            if (target.getPoseAmbiguity() <= .2) {
              previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
              m_DrivePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          }
        } 
      }

        else {
            previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
            m_DrivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      }
      }
      );
    }

  }

  private boolean once = true;

  @Override
  public void periodic() {
    m_DrivePoseEstimator.update(m_drivetrain.getNavxAngle(), m_drivetrain.getPositions());

    if(m_PhotonPoseEstimatorBack != null){
      updateEachPoseEstimator(m_PhotonPoseEstimatorBack, 2);
    }

    if (m_PhotonPoseEstimatorFront != null){
      updateEachPoseEstimator(m_PhotonPoseEstimatorFront, 1);
    }

    m_field.setRobotPose(getCurrentPose());
    //  SmartDashboard.putNumber("PoseEstimator X", getCurrentPose().getX());
    //  SmartDashboard.putNumber("PoseEstimator Y", getCurrentPose().getY());
    //  SmartDashboard.putNumber("PoseEstimator Angle", getCurrentPose().getRotation().getDegrees());
  }


  public Pose2d getCurrentPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    m_DrivePoseEstimator.resetPosition(m_drivetrain.getNavxAngle(), m_drivetrain.getPositions(), newPose);
  }

}