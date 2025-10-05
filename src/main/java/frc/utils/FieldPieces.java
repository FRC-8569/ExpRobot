package frc.utils;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;

public enum FieldPieces {
    Processor(3),
    BargeBlue(4),
    BargeRed(5),
    ReefAB(7),
    ReefCD(8),
    ReefEF(9),
    ReefGH(10),
    ReefIJ(11),
    ReefKL(6);

    int tag;
    FieldPieces(int tag){
        this.tag = tag;
    }

    private static Pose2d getPose(int tagID){
        return GlobalConstants.field.getTagPose(tagID).orElseThrow().toPose2d();
    }

    private Pose2d getItemPose(){
        Drivetrain.getInstance().withNowDoing("開到%s".formatted(this.toString()));
        return switch(this){
            case Processor -> getPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 3 : 16);
            case BargeBlue -> getPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 4 : 14);
            case BargeRed -> getPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 5: 15);
            default -> getPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? tag : tag+11);
        };
    }

    public Pose2d withReefSide(ReefSide side){
        Drivetrain.getInstance().withNowDoing("開到%s的%s".formatted(this.toString(), side == ReefSide.Left ? "左邊" : "右邊"));
        return getItemPose().plus(side.getOffset());
    }

    public enum ReefSide{
        Left,
        Right;

        public Transform2d getOffset(){
            return new Transform2d(Centimeters.of(0), Centimeters.of((32.862 /2)*(this == Left ? -1 : 1)), Rotation2d.kZero);
        }
    }

    public enum IntergratedField{
        CoralStation,
        Reef;

        public Pose2d getNearest(Pose2d currentPose) {
            return switch (this) {
                case CoralStation -> currentPose.nearest(List.of(
                    getPose(1), getPose(2), getPose(11), getPose(12)
                ));
                case Reef -> currentPose.nearest(
                    List.of(FieldPieces.ReefAB, FieldPieces.ReefCD, FieldPieces.ReefEF,
                            FieldPieces.ReefGH, FieldPieces.ReefIJ, FieldPieces.ReefKL)
                        .stream()
                        .flatMap(r -> Stream.of(ReefSide.Left, ReefSide.Right)
                            .map(side -> r.getItemPose().plus(side.getOffset())))
                        .toList()
                );
    };
}
    }
}
