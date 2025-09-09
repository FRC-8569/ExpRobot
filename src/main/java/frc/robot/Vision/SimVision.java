package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;

public class SimVision implements Vision{
    private PhotonCamera LocalizationCamera;
    public PhotonCameraSim LocalizationCameraSim;
    private SimCameraProperties prop;
    public VisionSystemSim SimSystem;
    public Notifier SimNotifer;
    public DoublePublisher SimulateFPS;
    public StructArrayPublisher<Pose3d> SeeableTarget;
    public StructPublisher<Pose3d> CameraPose, RobotPose;

    public SimVision(){
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCamera);
        prop = new SimCameraProperties();
        prop.setCalibration(1280, 720, Rotation2d.fromDegrees(55));
        prop.setFPS(30);
        LocalizationCameraSim = new PhotonCameraSim(LocalizationCamera, prop, Constants.Field);
        SimSystem = new VisionSystemSim("main");
        SimSystem.addAprilTags(Constants.Field);
        SimSystem.addCamera(LocalizationCameraSim, new Transform3d(Pose3d.kZero, Constants.CameraPose));
        LocalizationCameraSim.enableDrawWireframe(true);
        SimulateFPS = NetworkTableInstance.getDefault().getDoubleTopic("Vision/SimFPS").publish();
        SeeableTarget = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Target", Pose3d.struct).publish();
        CameraPose = NetworkTableInstance.getDefault().getStructTopic("Vision/CameraPose", Pose3d.struct).publish();
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Vision/RawPose", Pose3d.struct).publish();
    }

    private Pose3d[] getTargets(){
        List<Pose3d> target = new ArrayList<Pose3d>();
        var results = LocalizationCamera.getAllUnreadResults();
        if(results.isEmpty()) return null;
        for(var res : results.get(results.size() - 1).targets){
            target.add(SimSystem.getCameraPose(LocalizationCameraSim).orElseThrow().transformBy(res.getBestCameraToTarget()));
        }
        return target.toArray(Pose3d[]::new);
    } 

    @Override
    public Pose2d getPose(){
        return null;
    }


    @Override
    public void update(Pose2d pose){
        SimSystem.update(pose);
        SeeableTarget.accept(getTargets());
        CameraPose.accept(SimSystem.getCameraPose(LocalizationCameraSim).orElseThrow());
        RobotPose.accept(new Pose3d(pose));
    }
}
