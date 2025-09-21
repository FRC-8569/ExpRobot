package frc.robot.Elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
public class SimElevator {
    public SparkMax motors;
    public SparkMaxSim sims;
    public SparkRelativeEncoderSim encoders;
    public ElevatorSim elevatorSim;

    public SimElevator(){
        motors = RealElevator.system().LeftMotor;
        sims = new SparkMaxSim(motors, DCMotor.getNEO(2));
        encoders = sims.getRelativeEncoderSim();
        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2), 
            Constants.GearRatio, 
            15, 
            34.1/2/100, 
            0, 
            180, 
            false, 
            0, 
            null);
    }

    public void elevate(double height){
        elevatorSim.setState(encoders.getPosition(), encoders.getVelocity());
    }
}
