package frc.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.InputMismatchException;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Auto.Constants.FieldPieces;
import frc.robot.Auto.Constants.ReefSide;
import frc.robot.Elevator.Constants.ReefHeight;

public class ReefState extends SubsystemBase {
    public List<Boolean> AutoL2, AutoL3, AutoL4;
    public List<Boolean> TeleOpL2, TeleOpL3, TeleOpL4;
    public List<Boolean> RevealL2, RevealL3, RevealL4;
    public Supplier<Boolean> isAuto;
    public BooleanArrayPublisher L2Pub, L3Pub, L4Pub;

    public static ReefState state;

    public ReefState() {
        AutoL2 = new ArrayList<Boolean>(Collections.nCopies(12, false));
        AutoL3 = AutoL2;
        AutoL4 = AutoL2;
        TeleOpL2 = AutoL2;
        TeleOpL3 = AutoL2;
        TeleOpL4 = AutoL2;
        RevealL2 = AutoL2;
        RevealL3 = AutoL2;
        RevealL4 = AutoL2;

        L2Pub = NetworkTableInstance.getDefault().getBooleanArrayTopic("FullState/Reef/L2").publish();
        L3Pub = NetworkTableInstance.getDefault().getBooleanArrayTopic("FullState/Reef/L3").publish();
        L4Pub = NetworkTableInstance.getDefault().getBooleanArrayTopic("FullState/Reef/L4").publish();
    }

    public void scoreCoral(FieldPieces place, ReefSide side, ReefHeight height) {
        if (isAuto.get()) {
            switch (height) {
                case L2 -> AutoL2.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                case L3 -> AutoL3.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                case L4 -> AutoL4.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                default -> throw new InputMismatchException("Not Support lah");
            }
        } else {
            switch (height) {
                case L2 -> TeleOpL2.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                case L3 -> TeleOpL3.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                case L4 -> TeleOpL4.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
                default -> throw new InputMismatchException("Not Support lah");
            }
        }
        switch (height) {
            case L2 -> RevealL2.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
            case L3 -> RevealL3.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
            case L4 -> RevealL4.set(place.toInt() + (side == ReefSide.Right ? 1 : 0), true);
            default -> throw new InputMismatchException("Not Support lah");
        }
    }

    public double getScore() {
        return AutoL2.stream().filter(b -> b).count() * 4 + 
               AutoL3.stream().filter(b -> b).count() * 6 + 
               AutoL4.stream().filter(b -> b).count() * 7 + 
               TeleOpL2.stream().filter(b -> b).count() * 3+ 
               TeleOpL3.stream().filter(b -> b).count() * 4 + 
               TeleOpL4.stream().filter(b -> b).count() * 5;
    }

    public void withAutoSupplier(Supplier<Boolean> isAuto) {
        this.isAuto = isAuto;
    }

    public boolean[][] getLevel(){
        boolean[] L2 = new boolean[12], L3 = new boolean[12], L4 = new boolean[12];
        for(int i = 0; i < 12; i++){
            L2[i] = this.RevealL2.get(i);
            L3[i]  = this.RevealL3.get(i);
            L4[i] = this.RevealL4.get(i);
        }
        return new boolean[][]{L2,L3,L4};
    }

    public static ReefState getInstance() {
        if (state == null)
            state = new ReefState();
        return state;
    }

    public void periodic(){   
        L2Pub.accept(getLevel()[0]);
        L3Pub.accept(getLevel()[1]);
        L4Pub.accept(getLevel()[2]);
    }
}