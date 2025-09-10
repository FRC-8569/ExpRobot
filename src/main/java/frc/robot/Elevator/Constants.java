package frc.robot.Elevator;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Constants {
    public static final int LeftID = 50;
    public static final int RightID = 51;
    public static final int ShooterID = 52;

    public static final double PositionConvertionFactor = 3.41 * Math.PI;
    public static final double VelocityConvertionFactor = PositionConvertionFactor / 60;
    public static final int CurrentLimit = 30;
    public static final double GearRatio = 10.71;

    public static final ClosedLoopConfig PID = new ClosedLoopConfig()
            .pidf(0.255, 0, 0.05, 1.0 / 473)
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

    public enum ReefHeight {
        L0(0), // Intake Mode
        L1(102), // 102
        L2(150),
        L3(214);

        private double height;

        ReefHeight(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

        public ReefHeight next() {
            return switch(this){
                case L0 -> L1;
                case L1 -> L2;
                case L2 -> L3;
                case L3 -> L0;
            };
        }

        @Override
        public String toString(){
            return switch(this){
                case L0 -> "降到L0";
                case L1 -> "升到L1";
                case L2 -> "升到L2";
                case L3 -> "升到L3";
            };
        }
    }
}
