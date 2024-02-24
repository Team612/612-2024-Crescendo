// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeAndPivot;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveModule;
// import frc.robot.subsystems.Graboloid Jr. XIV
public class ShuffleBoardButtons {
    PhotonCamera cam = new PhotonCamera("camName"); // Whatever the camera name is plug it in here
    Vision m_vision = new Vision(cam);
    SwerveModule m_swerve = new SwerveModule(0, null);
    Drivetrain m_drivetrain = new Drivetrain();
    IntakeAndPivot m_intakeAndPivot = new IntakeAndPivot();
    Climb m_climb = new Climb();
    PoseEstimator m_poseEstimator = new PoseEstimator();
    Shooter m_shooter = new Shooter();
    ShuffleboardTab m_driverTab;
    ShuffleboardTab m_encoderTab;
    ShuffleboardTab m_graphTab;
    ShuffleboardTab m_debugTab;
    ShuffleboardTab m_limitSwitchTab;
    
    GenericEntry NavxAngle;
    GenericEntry PoseEstimatorAngle;
    GenericEntry PoseEstimatorX;
    GenericEntry PoseEstimatorY;
    GenericEntry fodState;
    // GenericEntry grabberCurrentGraph;
    GenericEntry telescopeCurrentGraph;
    GenericEntry BoreEncoders;
    GenericEntry pivotEntry;
    GenericEntry telescopeEntry;
    GenericEntry telescopeEncoderRate;
    GenericEntry PivPos;
    GenericEntry AprilDet;
    GenericEntry NoteDet;
    GenericEntry voltTurn;
    GenericEntry voltDrive;
    // GenericEntry isReleasing;

    GenericEntry pivotBLS;
    GenericEntry telescopeLimitSwitch;

    GenericEntry isBlueAlliance;

    public static GenericEntry toggleAlliance;
    

    //accessable entires
    public static GenericEntry grabberSpikeTresh;
    public static GenericEntry teleEncoderRateThresh;

    public static GenericEntry teleSpikeThresh;

    interface StringFunction {
        String run();
    }

    interface IntFunction {
        int run();
    }
 
    interface DoubleFunction {
        Double run();
    }
    
    interface BooleanFunction {
        Boolean run();
    }
    
    interface Pose2DFunction {
        Pose2d run();
    }

    interface DSVFunction {
        DoubleSolenoid.Value run();
    }

    // //debugging for arm thresholds
    public static GenericEntry lowGeneral;
    public static GenericEntry noteIr;
    public static GenericEntry climbPos;
    public static GenericEntry shoot;
    public static GenericEntry grab;
    // public static GenericEntry NoteDet;
    // public static GenericEntry voltTurn;
    // public static GenericEntry voltDrive;
    public static GenericEntry humanStation; //telescope debug 

    public void initButtons(){
        m_driverTab = Shuffleboard.getTab("DriverTab");
        m_encoderTab = Shuffleboard.getTab("Encoder");
        m_graphTab = Shuffleboard.getTab("Graphs");
        m_debugTab = Shuffleboard.getTab("Debug Tab");
        m_limitSwitchTab = Shuffleboard.getTab("Limit Switch Tab");
   
        // Srikar 
        DoubleFunction navXReturnO = () -> m_drivetrain.getNavxAngleO();
        Pose2DFunction poseEstimatorAngleO = () -> m_poseEstimator.getCurrentPose();
        BooleanFunction isAprilTagDetectedO = () -> m_vision.hasBestTarget(); // Joel
        m_vision.setPipeline(1); // set the vision pipeline to whatever the note pipeline is
        BooleanFunction isNoteDetectedO = () -> m_vision.hasBestTarget();; // Srikar
        DoubleFunction voltAngle1 = () -> m_swerve.getAngleVoltage(); // Achyut
        DoubleFunction voltDrive1 = () -> m_swerve.getDriveVoltage(); // Achyut

        AprilDet = m_debugTab.add("climbPos", isAprilTagDetectedO.run()).getEntry();
        NoteDet =  m_debugTab.add("isShooterShooting", isNoteDetectedO.run()).getEntry();
        voltTurn = m_debugTab.add("isGrabberGrabbing", voltAngle1.run()).getEntry();
        voltDrive = m_debugTab.add("isGrabberGrabbing", voltDrive1.run()).getEntry();

        
        // IntFunction PoseEstimatorXO = () -> m_poseEstimator.getCurrentPose();
        // IntFunction PoseEstimatorYO = () -> m_poseEstimator.getCurrentPose();

        // Joel
        BooleanFunction isBlueAllianceO = () -> true;
        BooleanFunction noteIrO = () -> m_intakeAndPivot.irSensorDetected();
        DSVFunction climbPosO = () -> m_climb.get();
        DoubleFunction shootO = () -> m_shooter.get();

        // Isaac and Achyut
        DoubleFunction grabO = () -> m_intakeAndPivot.getIntake();
        // IntFunction humanStationO = () -> 1;
        DoubleFunction PivPosO = () -> m_intakeAndPivot.getArm();
        // IntFunction fodState0 = () -> 1;
        // IntFunction pivotEntry0 = () -> 1;
        // IntFunction boreEncodersO = () -> 1;
        
        System.out.println(navXReturnO.run());
        System.out.println(poseEstimatorAngleO.run());

        //debug entries

        NavxAngle = m_debugTab.add("NavX angle", navXReturnO.run()).getEntry();
        PoseEstimatorAngle = m_debugTab.add("PoseEstimator Angle", poseEstimatorAngleO.run()).getEntry();
        // PoseEstimatorX = m_debugTab.add("PoseEstimator X", PoseEstimatorXO.run()).getEntry();
        // PoseEstimatorY = m_debugTab.add("PoseEstimator Y", PoseEstimatorYO.run()).getEntry();
        isBlueAlliance = m_debugTab.add("isBlueAlliance", isBlueAllianceO.run()).getEntry();

        // Game entries
        lowGeneral = m_debugTab.add("low General", 0.0).getEntry();
        noteIr = m_debugTab.add("noteir", noteIrO.run()).getEntry();
        climbPos = m_debugTab.add("climbPos", climbPosO.run()).getEntry();
        shoot =  m_debugTab.add("isShooterShooting", shootO.run()).getEntry();
        grab = m_debugTab.add("isGrabberGrabbing", grabO.run()).getEntry();
        
        // humanStation = m_debugTab.add("humanStation", humanStationO.run()).getEntry();

        // smartdashboard entries 
        
        // fodState = m_driverTab.add("FOD state?", fodState0.run()).getEntry();
        PivPos = m_driverTab.add("isPivotInIntake", PivPosO.run()).getEntry();


        // encoder entries
        // pivotEntry = m_encoderTab.add("Pivot Encoder", pivotEntry0.run()).getEntry();
        // BoreEncoders = m_encoderTab.add("Bore Encoder", boreEncodersO.run()).getEntry();

        //graphing entries
        // grabberSpikeTresh = m_graphTab.add("GrabberSpikeTresh", 0.0).getEntry();
        // grabberCurrentGraph = m_graphTab.add("Grabber Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        // telescopeCurrentGraph = m_graphTab.add("Telescope Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        // telescopeEncoderRate = m_graphTab.add("Telescope Encoder Rate", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        // telescopeEntry = m_encoderTab.add("Telescope Encoder", 0.0).getEntry();
        // teleEncoderRateThresh = m_graphTab.add("TeleEncoderThresh", 0.0).getEntry();
        // teleSpikeThresh = m_graphTab.add("TeleSpikeThresh", 0.0).getEntry();
        // pivotBLS = m_limitSwitchTab.add("pivotLimitSwitchState", false).getEntry();

        // telescopeLimitSwitch = m_limitSwitchTab.add("telescopeLimitSwitch", false).getEntry();
        // toggleAlliance = m_driverTab.add("Is Red Alliance?",false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    // @Override
    public void periodoic() {
      // This method will be called once per scheduler run
    }
    
}