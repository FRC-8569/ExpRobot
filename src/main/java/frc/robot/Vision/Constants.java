package frc.robot.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final String LocalizationCameraName = "LocalizationCamera";
    public static final Pose3d LocalizationCameraPlace = new Pose3d(Millimeters.of(265.55762),Millimeters.of(-310.65875),Millimeters.of(237.85235), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
    public static final String DriverCameraName = "DriverCamera";
    public static final PoseStrategy Strategy = org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    
}
