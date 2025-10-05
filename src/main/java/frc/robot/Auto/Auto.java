package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.Drivetrain;
import frc.utils.FieldPieces;
import frc.utils.FieldPieces.ReefSide;

public class Auto {
    static Drivetrain drivetrain = Drivetrain.getInstance();

    public static Command getAuto(){
        return drivetrain.drive(FieldPieces.ReefAB.withReefSide(ReefSide.Left));
    }
}
