package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class Elevator {
    public static Elevator elevator;
    public SparkMax LeftMotor, RightMotor;
    public RelativeEncoder MotionEncoder;
    public SparkClosedLoopController MotionPID;
}
