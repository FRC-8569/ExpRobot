package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Constants.SwerveMod;
import frc.robot.Vision.Vision;
import frc.utils.FieldPieces.IntergratedField;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    public Vision vision;
    public ApplyRobotSpeeds AutoDrive;
    public static Drivetrain drivetrain;
    public SimDrive simDrive;
    public boolean isOperaterPerspectiveApplied = false;
    public String NowDoing = "null";

    private List<DoublePublisher> MotorTorque;
    private DoublePublisher TotoalForce;

    // ? Simulations
    private Time SimLoop = Milliseconds.of(5);
    public Notifier SimNotifier;


    private Drivetrain(SwerveDrivetrainConstants DrivetrainConstants, SwerveModuleConstants<?,?,?>... modules){

        super(TalonFX::new, TalonFX::new, CANcoder::new, Constants.DrivetrainConstants,
            Utils.isSimulation() ? SimDrive.regulateModuleConstantsForSimulation(modules) : modules);

        MotorTorque = List.of(
            NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/Details/MotorTorque/FrontLeft").publish(),
            NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/Details/MotorTorque/FrontRight").publish(),
            NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/Details/MotorTorque/BackLeft").publish(),
            NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/Details/MotorTorque/BackRight").publish()
        );
        TotoalForce = NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/RobotForce").publish();

        AutoDrive = new ApplyRobotSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position);

        vision = Vision.getInstance();

        autoInit();
        if(Utils.isSimulation()) simInit();
    }

    public Command drive(Supplier<SwerveRequest> req){
        return run(() -> setControl(req.get()));
    }

    public Command drive(Pose2d pose){
        try{
            return new PathfindingCommand(
            pose.plus(new Transform2d(Meters.of(0), Constants.BumperSize.div(2), Rotation2d.kZero)), 
            PathConstraints.unlimitedConstraints(12), 
            () -> getState().Pose, 
            () -> getState().Speeds, 
            (speeds, ff) -> setControl(AutoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())), 
            new PPHolonomicDriveController(
                    new PIDConstants(4, 0, 0),
                    new PIDConstants(8, 0, 0.02)), 
            RobotConfig.fromGUISettings(), 
            this);
        }catch(Exception e){
            DriverStation.reportWarning("Error generating Path: "+e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command drive(IntergratedField pieces){
        NowDoing = "開到最近的%s".formatted(pieces.toString());
        return drive(pieces.getNearest(getState().Pose));
    }

    public void withNowDoing(String nowdoing){
        this.NowDoing = nowdoing;
    }

    @Override
    public void periodic(){
        double force = 0;
        for(int i = 0; i < 4; i++){
            MotorTorque.get(i).accept(getModule(i).getDriveMotor().getMotorKT().getValueAsDouble()*getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
            force += getModule(i).getDriveMotor().getMotorKT().getValueAsDouble()*getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble()/Constants.WheelRadius.in(Meters)/9.8067;
        }
        TotoalForce.accept(force);
        Pose2d pose = vision.getPose();
        if(pose != null) addVisionMeasurement(pose, Utils.getCurrentTimeSeconds());
    
        if(!isOperaterPerspectiveApplied || DriverStation.isDisabled()){
            setOperatorPerspectiveForward(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
            isOperaterPerspectiveApplied = true;
        }
    }

    private void autoInit(){
        try{
            AutoBuilder.configure(
                () -> getState().Pose, 
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, ff) -> setControl(AutoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())), 
                new PPHolonomicDriveController(
                    new PIDConstants(4, 0, 0),
                    new PIDConstants(8, 0, 0.02)), 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Red, 
                this);
        }catch(Exception e){
            DriverStation.reportError("Error initializting PathPlanner: "+e.getMessage(), e.getStackTrace());
        }
    }

    @SuppressWarnings("unchecked")
    private void simInit(){
        simDrive = new SimDrive(
            SimLoop, 
            Constants.RobotWeight, 
            Constants.BumperSize, 
            Constants.BumperSize, 
            DCMotor.getKrakenX60Foc(1), 
            DCMotor.getKrakenX60(1), 
            1.0, 
            getModuleLocations(), 
            getPigeon2(), 
            getModules(), 
            SwerveMod.FrontLeft.constants, SwerveMod.FrontRight.constants, 
            SwerveMod.BackLeft.constants, SwerveMod.BackRight.constants);

        SimNotifier = new Notifier(simDrive::update);
        SimNotifier.setName("DrivetrainSimNotifier");
        SimNotifier.startPeriodic(SimLoop.in(Seconds));
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null) drivetrain = new Drivetrain(Constants.DrivetrainConstants, SwerveMod.FrontLeft.constants, SwerveMod.FrontRight.constants, SwerveMod.BackLeft.constants, SwerveMod.BackRight.constants);
        return drivetrain;
    }
}
