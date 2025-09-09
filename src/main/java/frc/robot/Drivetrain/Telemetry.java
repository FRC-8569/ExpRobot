package frc.robot.Drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {
    private final StructPublisher<ChassisSpeeds> DriveSpeeds;
    private final StructArrayPublisher<SwerveModuleState> TargetState, CurrentState;
    private final StructPublisher<Pose2d> CurrentPose;

    public Telemetry(){
        CurrentPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/Posse", Pose2d.struct).publish();
        DriveSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/ChassisSpeeds", ChassisSpeeds.struct).publish();
        TargetState = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/TargetState", SwerveModuleState.struct).publish();
        CurrentState = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/CurrentState", SwerveModuleState.struct).publish();
    }

    public void telemerize(SwerveDriveState state){
        DriveSpeeds.accept(state.Speeds);
        CurrentState.accept(state.ModuleStates);
        TargetState.accept(state.ModuleTargets);
        CurrentPose.accept(state.Pose);
    }
}
