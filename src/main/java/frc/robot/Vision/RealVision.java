package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class RealVision implements Vision{
    public PhotonCamera DriverCamera, LocalizationCamera;
    public PhotonPoseEstimator PoseEstmator;
    public StructPublisher<Pose3d> RawPose;
    public BooleanPublisher isPhotonUpdated;
    public StructArrayPublisher<Transform3d> Transoforms;
    public StringPublisher FPSState;
    public StructPublisher<Pose3d> CameraPose;
    
    public RealVision(){
        DriverCamera = new PhotonCamera(Constants.DriverCamera);
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCamera);
        PoseEstmator = new PhotonPoseEstimator(Constants.Field, Constants.Strategy, new Transform3d(Pose3d.kZero, Constants.CameraPose));

        RawPose = NetworkTableInstance.getDefault().getStructTopic("Vision/RawPose", Pose3d.struct).publish();
        isPhotonUpdated = NetworkTableInstance.getDefault().getBooleanTopic("Vision/isPhotonUpdated").publish();
        isPhotonUpdated.setDefault(false);
        FPSState = NetworkTableInstance.getDefault().getStringTopic("Vision/FPSState").publish();
        Transoforms = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Targets", Transform3d.struct).publish();
        Transoforms.setDefault(new Transform3d[]{Transform3d.kZero});
        CameraPose = NetworkTableInstance.getDefault().getStructTopic("Vision/CameraPose", Pose3d.struct).publish();
    }

    @Override
    public Pose2d getPose(){
        if(!LocalizationCamera.isConnected()) {
            isPhotonUpdated.accept(false);
            return null;
        } else {
            var result = LocalizationCamera.getAllUnreadResults();
            try{
                Pose3d pose = PoseEstmator.update(result.get(result.size() - 1)).orElseThrow().estimatedPose;
                List<Pose3d> t = new ArrayList<Pose3d>();
                RawPose.accept(pose);
                for(var tar : result.get(result.size() - 1).targets){
                    t.add(pose.plus(new Transform3d(Constants.CameraPose.toMatrix())).transformBy(tar.getAlternateCameraToTarget()));
                }
                Transoforms.accept(t.toArray(Transform3d[]::new));

                return pose.toPose2d();
            }catch(Exception e){
                isPhotonUpdated.accept(false);
                return null;
            }
        }
    }

    @Override
    public void update(Pose2d pose){
    }


    public void withPose(Supplier<Pose2d> pose){}
}
