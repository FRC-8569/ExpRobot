package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface Vision {
    Pose2d getPose();
    void update(Pose2d pose); 
}
