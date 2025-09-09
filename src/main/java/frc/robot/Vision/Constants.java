package frc.robot.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final String LocalizationCamera = "LocalizationCamera";
    public static final String DriverCamera = "DriverCamera";
    public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final PoseStrategy Strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final Pose3d CameraPose = new Pose3d(Millimeters.of(265.55762),Millimeters.of(-310.65875),Millimeters.of(237.85235), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
}
