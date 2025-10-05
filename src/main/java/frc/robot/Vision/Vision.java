package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;

public class Vision extends SubsystemBase{
    public PhotonCamera LocalizationCamera;
    public PhotonPoseEstimator PoseEstimator;

    public StructPublisher<Pose3d> RawPose3d;
    public StructArrayPublisher<Pose3d> VisionTargets;
    public StructSubscriber<Pose2d> RobotPose;
    public BooleanPublisher isPhotonUpdated;
    public static Vision vision;

    public PhotonCameraSim LocalizationCamraSim;
    public VisionSystemSim SimSystem;
    public Notifier SimNotifer;

    private Vision(){
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCameraName);
        PoseEstimator = new PhotonPoseEstimator(GlobalConstants.field, Constants.Strategy, new Transform3d(Constants.LocalizationCameraPlace.toMatrix()));
        RawPose3d = NetworkTableInstance.getDefault().getStructTopic("Vision/RawPose", Pose3d.struct).publish();
        VisionTargets = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Targets", Pose3d.struct).publish();
        isPhotonUpdated = NetworkTableInstance.getDefault().getBooleanTopic("Vision/isAvaliable").publish();
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).subscribe(null);
        if(RobotBase.isSimulation()) simInit();
    }

    public Pose2d getPose(){
        if(!LocalizationCamera.isConnected()) return null;
        var results = LocalizationCamera.getAllUnreadResults();
        if(results.size() < 1) return null;
        var res = PoseEstimator.update(results.get(results.size() - 1)).orElse(null);
        if(res == null) return null;
        RawPose3d.accept(res.estimatedPose);
        RobotPose.getAtomic(res.estimatedPose.toPose2d());
        List<Pose3d> t = new ArrayList<Pose3d>();
        res.targetsUsed.forEach(tar -> t.add(new Pose3d(tar.getBestCameraToTarget().toMatrix())));
        VisionTargets.accept(t.toArray(Pose3d[]::new));
        return res.estimatedPose.toPose2d();
    }

    private void simInit(){
        LocalizationCamraSim = new PhotonCameraSim(LocalizationCamera,SimCameraProperties.LL2_1280_720());
        SimSystem = new VisionSystemSim("SimVision");
        SimSystem.addCamera(LocalizationCamraSim, new Transform3d(Constants.LocalizationCameraPlace.toMatrix()));
        SimSystem.addAprilTags(GlobalConstants.field);
    }

    @Override
    public void simulationPeriodic(){
        SimSystem.update(Drivetrain.getInstance().getState().Pose);
    }

    @Override
    public void periodic(){
        isPhotonUpdated.accept(getPose() != null);
    }

    public static Vision getInstance(){
        if(vision == null) vision = new Vision();
        return vision;
    }
}
