// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants;
public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static Transform3d robotToCam;
  public PhotonPoseEstimator m_PoseEstimator;

  static Vision visionInstance = null;

  PhotonCamera camera;

  private Pose2d robotInTagPose;
  /**
   * Creates a new Vision.
   * 
   * @throws IOException
   **/

  public Vision(PhotonCamera camera) {
    // tag 1
    final Translation3d translation1 = new Translation3d(15.079472, 0.245872, 1.355852);
    final Quaternion q1 = new Quaternion(0.5, 0, 0, 0.8660254037844386);
    final Rotation3d rotation1 = new Rotation3d(q1);
    final AprilTag tag1 = new AprilTag(1, new Pose3d(translation1, rotation1));

    // tag 2
    final Translation3d translation2 = new Translation3d(16.185134, 0.883666, 1.355852);
    final Quaternion q2 = new Quaternion(0.5, 0, 0, 0.8660254037844386);
    final Rotation3d rotation2 = new Rotation3d(q2);
    final AprilTag tag2 = new AprilTag(2, new Pose3d(translation2, rotation2));

    // tag 3
    final Translation3d translation3 = new Translation3d(16.579342, 4.982718, 1.451102);
    final Quaternion q3 = new Quaternion(0, 0, 0, 1);
    final Rotation3d rotation3 = new Rotation3d(q3);
    final AprilTag tag3 = new AprilTag(3, new Pose3d(translation3, rotation3));

    // tag 4
    final Translation3d translation4 = new Translation3d(16.579342, 5.547868, 1.451102);
    final Quaternion q4 = new Quaternion(0, 0, 0, 1);
    final Rotation3d rotation4 = new Rotation3d(q4);
    final AprilTag tag4 = new AprilTag(4, new Pose3d(translation4, rotation4));

    // tag 5
    final Translation3d translation5 = new Translation3d(14.700758, 8.2042, 1.355852);
    final Quaternion q5 = new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476);
    final Rotation3d rotation5 = new Rotation3d(q5);
    final AprilTag tag5 = new AprilTag(5, new Pose3d(translation5, rotation5));

    // tag 6
    final Translation3d translation6 = new Translation3d(1.8415, 8.2042, 1.355852);
    final Quaternion q6 = new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476);
    final Rotation3d rotation6 = new Rotation3d(q6);
    final AprilTag tag6 = new AprilTag(6, new Pose3d(translation6, rotation6));

    // tag 7
    final Translation3d translation7 = new Translation3d(-0.0381, 5.547868, 1.4511);
    final Quaternion q7 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation7 = new Rotation3d(q7);
    final AprilTag tag7 = new AprilTag(7, new Pose3d(translation7, rotation7));

    // tag 8
    final Translation3d translation8 = new Translation3d(-0.0381, 4.982718, 1.451102);
    final Quaternion q8 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation8 = new Rotation3d(q8);
    final AprilTag tag8 = new AprilTag(8, new Pose3d(translation8, rotation8));

    // tag 9
    final Translation3d translation9 = new Translation3d(0.356108, 0.883666, 1.355852);
    final Quaternion q9 = new Quaternion(0.8660254037844387, 0, 0, 0.5);
    final Rotation3d rotation9 = new Rotation3d(q9);
    final AprilTag tag9 = new AprilTag(9, new Pose3d(translation9, rotation9));

    // tag 10
    final Translation3d translation10 = new Translation3d(1.461516, 0.245872, 1.355852);
    final Quaternion q10 = new Quaternion(0.8660254037844387, 0, 0, 0.5);
    final Rotation3d rotation10 = new Rotation3d(q10);
    final AprilTag tag10 = new AprilTag(10, new Pose3d(translation10, rotation10));

    // tag 11
    final Translation3d translation11 = new Translation3d(11.904726, 3.713226, 1.3208);
    final Quaternion q11 = new Quaternion(-0.8660254037844387, 0, 0, 0.5);
    final Rotation3d rotation11 = new Rotation3d(q11);
    final AprilTag tag11 = new AprilTag(11, new Pose3d(translation11, rotation11));

    // tag 12
    final Translation3d translation12 = new Translation3d(11.904726, 4.49834, 1.3208);
    final Quaternion q12 = new Quaternion(0.8660254037844387, 0, 0, 0.5);
    final Rotation3d rotation12 = new Rotation3d(q12);
    final AprilTag tag12 = new AprilTag(12, new Pose3d(translation12, rotation12));

    // tag 13
    final Translation3d translation13 = new Translation3d(11.220196, 4.105148, 1.3208);
    final Quaternion q13 = new Quaternion(0, 0, 0, 1);
    final Rotation3d rotation13 = new Rotation3d(q13);
    final AprilTag tag13 = new AprilTag(13, new Pose3d(translation13, rotation13));

    // tag 14
    final Translation3d translation14 = new Translation3d(5.320792, 4.105148, 1.3208);
    final Quaternion q14 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation14 = new Rotation3d(q14);
    final AprilTag tag14 = new AprilTag(14, new Pose3d(translation14, rotation14));

    // tag 15
    final Translation3d translation15 = new Translation3d(4.641342, 4.49834, 1.3208);
    final Quaternion q15 = new Quaternion(0.5, 0, 0, 0.8660254037844386);
    final Rotation3d rotation15 = new Rotation3d(q15);
    final AprilTag tag15 = new AprilTag(15, new Pose3d(translation15, rotation15));

    // tag 16
    final Translation3d translation16 = new Translation3d(4.641342, 3.713226, 1.3208);
    final Quaternion q16 = new Quaternion(-0.5, 0, 0, 0.8660254037844387);
    final Rotation3d rotation16 = new Rotation3d(q16);
    final AprilTag tag16 = new AprilTag(8, new Pose3d(translation16, rotation16));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    
    atList.add(tag1);
    atList.add(tag2);
    atList.add(tag3);
    atList.add(tag4);
    atList.add(tag5);
    atList.add(tag6);
    atList.add(tag7);
    atList.add(tag8);
    atList.add(tag9);
    atList.add(tag10);
    atList.add(tag11);
    atList.add(tag12);
    atList.add(tag13);
    atList.add(tag14);
    atList.add(tag15);
    atList.add(tag16);

    robotInTagPose = new Pose2d();
    this.camera = camera;
    resetRobotPose();

    aprilTagFieldLayout = new AprilTagFieldLayout(atList, 16.451 , 8.211 );

    robotToCam = new Transform3d(new Translation3d(0.09, 0.0, 0.3), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    
    m_PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, robotToCam);

    //m_PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    

  }

  public static Vision getVisionInstance() {
    if (visionInstance == null) {
      visionInstance = new Vision(new PhotonCamera(Constants.VisionConstants.cameraName));
    }
    return visionInstance;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  // getting the vision pose from the april tags
  public Pose2d getTagPose() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();

      Transform3d tagSpace = bestTarget.getBestCameraToTarget();

      return new Pose2d(tagSpace.getX(), tagSpace.getY(), new Rotation2d( (bestTarget.getYaw()) * (Math.PI/180)) );
    }
    return new Pose2d();
  }

  public Pose2d getRobotPose(){
    Pose2d tagPose = robotInTagPose;
    return new Pose2d().transformBy(new Transform2d(tagPose.getTranslation(), tagPose.getRotation()));
  }

  public void resetRobotPose(){
    robotInTagPose = getTagPose();
  }

  // return tag pose
  public Pose3d return_tag_pose(int id) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    return pose_of_tag.get();
  }

  // self calculations
  public Pose3d return_camera_pose_tag(int id, PhotonPipelineResult results) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    Pose3d tag_pose = pose_of_tag.get();
    Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
    return tag_pose.plus(cameraTransform);
  }

  // photonvision pose estimator
  public Optional<EstimatedRobotPose> return_photon_pose(Pose2d latestPose) {
    m_PoseEstimator.setReferencePose(latestPose);
    return m_PoseEstimator.update();
  }

  public PhotonPoseEstimator getVisionPose(){
    return m_PoseEstimator;
  }
  
  

  @Override
  public void periodic() {
    if (camera.getDistCoeffs().equals(Optional.empty())){
      System.out.println("NO CALIBRATION");
    }

  }
}