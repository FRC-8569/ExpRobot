package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;

public class Telemetry extends SubsystemBase{
    public Drivetrain drivetrain;

    public StructPublisher<Pose2d> RobotPose;
    public StructArrayPublisher<SwerveModuleState> SwerveModuleStates;
    public StringPublisher DrivetrainNowDoing;

    public Telemetry(){
        drivetrain = Drivetrain.getInstance();

        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        SwerveModuleStates = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/SwerveModuleState", SwerveModuleState.struct).publish();

        DrivetrainNowDoing = NetworkTableInstance.getDefault().getStringTopic("RobotState/Drivetrain").publish();
    }

    private void NTupdate(){
        RobotPose.accept(drivetrain.getState().Pose);
        SwerveModuleStates.accept(drivetrain.getState().ModuleStates);
        DrivetrainNowDoing.accept(drivetrain.NowDoing);
    }

    @Override
    public void periodic(){
        NTupdate();
    }
}
