package frc.robot.Auto;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final AprilTagFieldLayout field = frc.robot.Vision.Constants.Field;
    public static final LinearVelocity MaxDriveVelocity = MetersPerSecond.of(3.5);
    public static final LinearAcceleration MaxAccel = MetersPerSecondPerSecond.of(9.8);
    public static final AngularVelocity MaxOmega = DegreesPerSecond.of(540);
    public static final AngularAcceleration MaxOmegaAccel = DegreesPerSecondPerSecond.of(720);
    public static final Distance RobotSize = Centimeters.of(80).div(2);
    public static final PathConstraints Constraints = new PathConstraints(MaxDriveVelocity, MaxAccel, MaxOmega,
            MaxOmegaAccel);

    public enum FieldPieces {
        CoralStation(2, 13),
        Processor(9, 16),
        ReefAB(7, 18),
        ReefCD(11, 17),
        ReefEF(6, 22),
        ReefGH(10, 21),
        ReefIJ(9, 20),
        ReefKL(8, 19);

        int[] tags;

        FieldPieces(int... TagIDs) {
            this.tags = TagIDs;
        }

        public Pose2d getItemPose(Pose2d pose, Alliance alliance) {
            if (this == CoralStation)
                return new Pose2d(
                        field.getTagPose(tags[alliance == Alliance.Red ? 0 : 1]).orElseThrow().toPose2d().getX(),
                        field.getTagPose(tags[alliance == Alliance.Red ? 0 : 1]).orElseThrow().toPose2d().getY(),
                        field.getTagPose(tags[alliance == Alliance.Red ? 0 : 1]).orElseThrow().toPose2d().getRotation()
                                .plus(Rotation2d.k180deg));
            else
                return field.getTagPose(tags[alliance == Alliance.Red ? 0 : 1]).orElseThrow().toPose2d();
        }

        public int toInt(){
            return switch (this) {
                case ReefAB -> 0;
                case ReefCD -> 2;
                case ReefEF -> 4;
                case ReefGH -> 6;
                case ReefIJ -> 8;
                case ReefKL -> 10;
                default -> -1;
            };
        }
    }

    public enum ReefSide {
        Left(),
        Right(),
        Null();

        ReefSide() {
        }

        public Transform2d getPose() {
            return switch (this) {
                case Left -> new Transform2d(Centimeters.of(0), Centimeters.of(-32.862 /2), Rotation2d.kZero);
                case Right -> new Transform2d(Centimeters.of(0), Centimeters.of(32.862 /2), Rotation2d.kZero);
                case Null -> Transform2d.kZero;
            };
        }

        @Override
        public String toString(){
            return switch(this){
                case Left -> "的左邊";
                case Right -> "的右邊";
                case Null -> "";
            };
        }
    }
}