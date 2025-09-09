package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Constants.FieldPieces;
import frc.robot.Auto.Constants.ReefSide;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.RealElevator;
import frc.robot.Elevator.Constants.ReefHeight;

public class Auto {
    public Drivetrain drivetrain;
    public RealElevator elevator;
    public static PathConstraints constraints;
    public boolean Timeout = true;

    public Auto() {
        drivetrain = Drivetrain.system();
        elevator = RealElevator.system();
        constraints = new PathConstraints(
                Constants.MaxDriveVelocity,
                Constants.MaxAccel,
                Constants.MaxOmega,
                Constants.MaxOmegaAccel);
    }

    public Auto(Drivetrain drivetrain, RealElevator elevator) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        constraints = new PathConstraints(
                Constants.MaxDriveVelocity,
                Constants.MaxAccel,
                Constants.MaxOmega,
                Constants.MaxOmegaAccel);
    }

    public Command getAuto() {
        return new SequentialCommandGroup(
                resetOdometry(),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.ReefKL, ReefSide.Left),
                        elevator.elevate(ReefHeight.L3)),
                elevator.spinShooter(false),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.CoralStation, ReefSide.Null),
                        elevator.elevate(ReefHeight.L0)),
                elevator.spinShooter(true),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.ReefKL, ReefSide.Right),
                        elevator.elevate(ReefHeight.L3)),
                elevator.spinShooter(false),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.CoralStation, ReefSide.Null),
                        elevator.elevate(ReefHeight.L0)),
                elevator.spinShooter(true),

                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.ReefAB, ReefSide.Left),
                        elevator.elevate(ReefHeight.L3)),
                elevator.spinShooter(false),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.CoralStation, ReefSide.Null),
                        elevator.elevate(ReefHeight.L0)),
                elevator.spinShooter(true),

                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.ReefAB, ReefSide.Right),
                        elevator.elevate(ReefHeight.L3)),
                elevator.spinShooter(false),
                new ParallelCommandGroup(
                        drivetrain.drive(FieldPieces.CoralStation, ReefSide.Null),
                        elevator.elevate(ReefHeight.L0)),
                elevator.spinShooter(true)

        ).withTimeout(Seconds.of(Timeout ? 15 : 135));
    }

    public Command resetOdometry() {
        return Commands.runOnce(() -> drivetrain.resetPose(DriverStation.getAlliance().orElseThrow() == Alliance.Blue
                ? new Pose2d(7.6, 6.15, Rotation2d.fromDegrees(180))
                : new Pose2d(9.97, 5, Rotation2d.fromDegrees(0))));
    }
}
