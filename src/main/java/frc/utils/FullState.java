package frc.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Auto.Constants.FieldPieces;
import frc.robot.Auto.Constants.ReefSide;
import frc.robot.Elevator.Constants.ReefHeight;

public class FullState implements Subsystem{
    public FieldPieces pieces = FieldPieces.CoralStation;
    public ReefSide side = ReefSide.Null;
    public Object height = "null";
    public double SpeedMode = 0.4;
    public static FullState state;
    public StringPublisher ElevatorState = NetworkTableInstance.getDefault().getStringTopic("RobotState/Elevator").publish(), 
                           ChassisState = NetworkTableInstance.getDefault().getStringTopic("RobotState/Chassis").publish();
    public FullState(){
    }

    public void withDrivetrainTarget(FieldPieces pieces, ReefSide side){
        this.pieces = pieces;
        this.side = side;
    }

    public void withElevatorTarget(ReefHeight height){
        this.height = height;
    }

    public void withShooterState(String shooter){
        this.height = shooter;
    }

    public static FullState getInstance(){
        if(state == null) FullState.state = new FullState();
        return state;
    }

    @Override
    public void periodic(){
        ChassisState.accept("開到%s%s".formatted(pieces.toString(), side.toString()));
        ElevatorState.accept(height.toString());
    }
}
