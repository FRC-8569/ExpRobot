package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Auto.Constants.FieldPieces;
import frc.robot.Auto.Constants.ReefSide;
import frc.robot.Vision.RealVision;
import frc.robot.Vision.SimVision;
import frc.robot.Vision.Vision;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static final Time SimLoop = Milliseconds.of(5);
    private Notifier SimNotifer = null;
    public Vision vision;
    private static SimDrive simDrive;
    private static boolean OperatorPerspectiveApplied = false;

    private SwerveRequest.ApplyRobotSpeeds AutoDrive;
    private DoublePublisher[] CANCoderPositions;

    public Drivetrain(SwerveDrivetrainConstants DrivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, DrivetrainConstants,
                Utils.isSimulation() ? SimDrive.regulateModuleConstantsForSimulation(modules) : modules);

        CANCoderPositions = new DoublePublisher[] {
                NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/debug/FLPosition").publish(),
                NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/debug/FRPosition").publish(),
                NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/debug/BLPosition").publish(),
                NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/debug/BRPosition").publish()
        };

        vision = Utils.isSimulation() ? new SimVision()
                : (NetworkTableInstance.getDefault().getTable("photonvision").getTopic("version").exists())
                        ? new RealVision()
                        : null;

        AutoDrive = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.Position)
                .withDesaturateWheelSpeeds(true);

        AutoInit();
        if (Utils.isSimulation())
            startSim();
        // NowDoing = new Alert("UpdatingAlerts","DrivetrainPlaceholder",
        // AlertType.kInfo);
        // NowDoing.set(true);
    }

    public Command drive(Supplier<SwerveRequest> req) {
        return run(() -> setControl(req.get()));
    }

    public Command drive(FieldPieces pieces, ReefSide side) {
        return drive(
                pieces.getItemPose(getState().Pose, DriverStation.getAlliance().orElseThrow()).transformBy(side.getPose()));
    }

    public Command drive(Pose2d CenterPose) {
        try {
            return new PathfindingCommand(
                    offsetPose(CenterPose),
                    frc.robot.Auto.Constants.Constraints,
                    () -> getState().Pose,
                    () -> getState().Speeds,
                    (speeds, ff) -> setControl(
                            AutoDrive
                                    .withSpeeds(speeds)
                                    .withDesaturateWheelSpeeds(true)
                                    .withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
                                    .withWheelForceFeedforwardsY(ff.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(4, 0, 0),
                            new PIDConstants(8, 0, 0.02)),
                    RobotConfig.fromGUISettings(),
                    this);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * offset pose from AprilTagFieldLayout raw output
     * 
     * @param pose
     * @return
     */
    private Pose2d offsetPose(Pose2d pose) {
        return pose.plus(new Transform2d(frc.robot.Auto.Constants.RobotSize, Centimeters.of(0), Rotation2d.k180deg));
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (simDrive != null)
            simDrive.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05);
        super.resetPose(pose);
    }

    @SuppressWarnings("unchecked")
    private void startSim() {
        simDrive = new SimDrive(
                SimLoop,
                Constants.RobotWeight,
                Constants.BumperSize,
                Constants.BumperSize,
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                Constants.WheelCoF,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                Constants.SwerveMod.FrontLeft.constants,
                Constants.SwerveMod.FrontRight.constants,
                Constants.SwerveMod.BackLeft.constants,
                Constants.SwerveMod.BackRight.constants);
        SimNotifer = new Notifier(simDrive::update);
        SimNotifer.setName("SimDrivetrainNotifer");
        SimNotifer.startPeriodic(SimLoop.in(Seconds));
    }

    @Override
    public void periodic() {
        if (!OperatorPerspectiveApplied || DriverStation.isDisabled())
            DriverStation.getAlliance().ifPresent(color -> setOperatorPerspectiveForward(
                    color == Alliance.Red ? Constants.RedAlliancePerspective : Constants.BlueAlliancePerspective));
        for (int i = 0; i < CANCoderPositions.length; i++)
            CANCoderPositions[i].accept(180.0 - getModule(i).getEncoder().getAbsolutePosition().getValue().in(Degrees));

        if (vision != null && vision.getPose() != null) {
            try {
                addVisionMeasurement(vision.getPose(), Utils.getCurrentTimeSeconds());
            } catch (Exception e) {
                DriverStation.reportWarning(
                        "Fucked up at updating Pose2d with PhotonVision with \n %s".formatted(e.getMessage()),
                        e.getStackTrace());
            }
        }

        vision.update(getState().Pose);
    }

    public void AutoInit() {
        try {
            AutoBuilder.configure(
                    () -> this.getState().Pose,
                    this::resetPose,
                    () -> this.getState().Speeds,
                    (speeds, ff) -> setControl(
                            AutoDrive
                                    .withSpeeds(speeds)
                                    .withDesaturateWheelSpeeds(true)
                                    .withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
                                    .withWheelForceFeedforwardsY(ff.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(4, 0, 0),
                            new PIDConstants(8, 0, 0.02)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().get() == Alliance.Red,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Fucked up at loading PathPlanner with \n %s".formatted(e.getMessage()),
                    e.getStackTrace());
        }
    }

    public static Drivetrain system() {
        return Constants.createDrivetrain();
    }

    // @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Command");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
                ".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
                ".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                null);
    }
}
