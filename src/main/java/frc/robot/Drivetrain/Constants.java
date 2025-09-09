package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {

    // ? PID Related
    public static final Slot0Configs DrivePID = new Slot0Configs()
        .withKP(4).withKI(0).withKD(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final Slot0Configs SteerPID = new Slot0Configs()
        .withKP(60).withKI(0).withKD(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final ClosedLoopOutputType DriveOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    public static final ClosedLoopOutputType SteerOutput = ClosedLoopOutputType.Voltage;
    public static final DriveMotorArrangement DriveMotor = DriveMotorArrangement.TalonFX_Integrated;
    public static final SteerMotorArrangement SteerMotor = SteerMotorArrangement.TalonFX_Integrated;
    public static final SteerFeedbackType SteerFeedback = SteerFeedbackType.FusedCANcoder;
    public static final Current SlipCurrent = Amps.of(55);
    public static final TalonFXConfiguration DriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration SteerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60)));
    

    // ? Some Configuration and Default Physical information
    public static final CANcoderConfiguration CANCoderConfig = new CANcoderConfiguration();
    public static final Pigeon2Configuration GyroConfig = null;
    public static final CANBus CANbus = new CANBus("", "./logs/example.hoot");
    public static final double CoupleRatio = 3.5714285714285716;;
    public static final double DriveGearRatio = 6.122448979591837;;
    public static final double SteerGearRatio = 21.428571428571427;;
    public static final Distance WheelRadius = Inches.of(2);
    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(5800.0/60/DriveGearRatio*WheelRadius.times(2).times(Math.PI).in(Meters));
    public static final AngularVelocity MaxOmega = RotationsPerSecond.of(1.25);
    public static final int GyroID = 1;

    // ? Simulations
    public static final MomentOfInertia DriveInteria = KilogramSquareMeters.of(0.01);
    public static final MomentOfInertia SteerInteria = DriveInteria;
    public static final Voltage DriveFrictionVoltage = Volts.of(0.2);
    public static final Voltage SteerFrictionVoltage = DriveFrictionVoltage;
    public static final Mass RobotWeight = Kilograms.of(50.41);
    public static final Distance BumperSize = Centimeters.of(80);
    public static final double WheelCoF = 1.0;

    // ? Some Constants but name very long

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(CANbus.getName())
        .withPigeon2Id(GyroID)
        .withPigeon2Configs(GyroConfig);
    
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantsCreator = 
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DriveGearRatio)
            .withSteerMotorGearRatio(SteerGearRatio)
            .withCouplingGearRatio(CoupleRatio)
            .withWheelRadius(WheelRadius)
            .withDriveMotorGains(DrivePID)
            .withSteerMotorGains(SteerPID)
            .withDriveMotorClosedLoopOutput(DriveOutput)
            .withSteerMotorClosedLoopOutput(SteerOutput)
            .withSlipCurrent(SlipCurrent)
            .withSpeedAt12Volts(MaxVelocity)
            .withDriveMotorType(DriveMotor)
            .withSteerMotorType(SteerMotor)
            .withFeedbackSource(SteerFeedback)
            .withDriveMotorInitialConfigs(DriveConfig)
            .withSteerMotorInitialConfigs(SteerConfig)
            .withEncoderInitialConfigs(CANCoderConfig)
            .withDriveInertia(SteerInteria)
            .withSteerInertia(DriveInteria)
            .withDriveFrictionVoltage(DriveFrictionVoltage)
            .withSteerFrictionVoltage(SteerFrictionVoltage);
            
    public class SwerveMod {
        public static final boolean LeftSideInverted = false;
        public static final boolean RightSideInverted = true;
        public static final boolean SteerMotorInverted = true;
        public static final boolean EncoderInverted = false;
        public static final double TrackWidth = Inches.of(10.61).in(Meters);
        
        public class FrontLeft {
            public static final int Drive = 31;
            public static final int Steer = 32;
            public static final int Encoder = 3;
            public static final Angle offset = Rotations.of(0.093017578125);
            
            public static final Translation2d place = new Translation2d(TrackWidth, TrackWidth);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants = 
                ConstantsCreator.createModuleConstants(Steer, Drive, Encoder, offset, place.getMeasureX(), place.getMeasureY(), LeftSideInverted, SteerMotorInverted, EncoderInverted);
        }

        public class FrontRight {
            public static final int Drive = 41;
            public static final int Steer = 42;
            public static final int Encoder = 4;
            public static final Angle offset = Rotations.of(0.2890625);
            
            public static final Translation2d place = new Translation2d(TrackWidth, -TrackWidth);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants = 
                ConstantsCreator.createModuleConstants(Steer, Drive, Encoder, offset, place.getMeasureX(), place.getMeasureY(), RightSideInverted, SteerMotorInverted, EncoderInverted);
        }

        public class BackLeft {
            public static final int Drive = 11;
            public static final int Steer = 12;
            public static final int Encoder = 10;
            public static final Angle offset = Rotations.of(-0.131103515625);
            
            public static final Translation2d place = new Translation2d(-TrackWidth, TrackWidth);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants = 
                ConstantsCreator.createModuleConstants(Steer, Drive, Encoder, offset, place.getMeasureX(), place.getMeasureY(), RightSideInverted, SteerMotorInverted, EncoderInverted);
        }

        public class BackRight {
            public static final int Drive = 21;
            public static final int Steer = 22;
            public static final int Encoder = 2;
            public static final Angle offset = Rotations.of(-0.094970703125);
            
            public static final Translation2d place = new Translation2d(-TrackWidth, -TrackWidth);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants = 
                ConstantsCreator.createModuleConstants(Steer, Drive, Encoder, offset, place.getMeasureX(), place.getMeasureY(), RightSideInverted, SteerMotorInverted, EncoderInverted);
        }
    }

    // ? Other Bullshits

    public static final Rotation2d BlueAlliancePerspective = Rotation2d.kZero;
    public static final Rotation2d RedAlliancePerspective = Rotation2d.k180deg;

    public static final Drivetrain createDrivetrain(){
        return new Drivetrain(DrivetrainConstants, SwerveMod.FrontLeft.constants, SwerveMod.FrontRight.constants, SwerveMod.BackLeft.constants, SwerveMod.BackRight.constants);
    }
}
