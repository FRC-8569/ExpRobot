package frc.robot.Elevator;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elevator.Constants.ReefHeight;
import frc.utils.FullState;

public class RealElevator extends SubsystemBase {
    public SparkMax LeftMotor, RightMotor, Shooter;
    public RelativeEncoder MotionEncoder;
    public SparkClosedLoopController MotionPID;
    private SparkMaxConfig LeftConfig, RightConfig, ShooterConfig;
    public StringPublisher CurrentHeight;
    public DoublePublisher PerciseHeight;

    public RealElevator(){
        LeftMotor = new SparkMax(Constants.LeftID, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.RightID, MotorType.kBrushless);
        Shooter = new SparkMax(Constants.ShooterID, MotorType.kBrushless);
        MotionEncoder = LeftMotor.getAlternateEncoder();
        MotionPID = LeftMotor.getClosedLoopController();

        CurrentHeight = NetworkTableInstance.getDefault().getStringTopic("Elevator/CurrentHeight").publish();
        PerciseHeight = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/PerciseHeight").publish();

        LeftConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();
        ShooterConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.CurrentLimit);

        LeftConfig.alternateEncoder
            .inverted(false)
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);

        LeftConfig.closedLoop.apply(Constants.PID);

        RightConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.CurrentLimit)
            .follow(LeftMotor, true);

        ShooterConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(20)
            .voltageCompensation(12);

        LeftMotor.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Shooter.configure(ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command elevate(ReefHeight height){
        return elevate(height.getHeight())
            .alongWith(Commands.runOnce(() -> FullState.getInstance().withElevatorTarget(height)));
    }

    private Command elevate(double height){
        return runOnce(() -> MotionPID.setReference(height, ControlType.kPosition));
    }

    public Command intake(){
        return elevate(ReefHeight.L0).andThen(spinShooter(true));
    }


    public Command spinShooter(boolean isIntake){
        return runEnd(
            () -> Shooter.set(isIntake ? 0.3 : 0.8), 
            () -> Shooter.stopMotor())
            .withTimeout(DriverStation.isAutonomous() ? isIntake ? Seconds.of(0.8) : Seconds.of(0.35) : Seconds.of(135))
            .alongWith(Commands.runOnce(() -> FullState.getInstance().withShooterState(isIntake ? "吃Coral" : "吐Coral")));
    }

    public ReefHeight getHeight(){
        double value = MotionEncoder.getPosition();
        for(ReefHeight h : ReefHeight.values()){
            if(Math.abs(h.getHeight() - value) < 3) return h;
        }
        return ReefHeight.L0;
    }

    public double getPerciseHeight(){
        return MotionEncoder.getPosition();
    }

    public static RealElevator system(){
        return new RealElevator();
    }

    @Override
    public void periodic(){
        CurrentHeight.accept(getHeight().toString().substring(getHeight().toString().length() - 2));
        PerciseHeight.accept(getPerciseHeight());
    }
}
