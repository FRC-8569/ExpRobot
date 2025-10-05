// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public XboxController controller = new XboxController(0);
  public Telemetry telemetry = new Telemetry();
  
  public RobotCentric driveReq = new RobotCentric()
    .withDeadband(Constants.MaxVelocity.times(0.05))
    .withRotationalDeadband(Constants.MaxOmega.times(0.05))
    .withDesaturateWheelSpeeds(true)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position);

  public RobotContainer() {
    if(!Utils.isSimulation()) drivetrain.setDefaultCommand(drivetrain.drive(() -> driveReq
      .withVelocityX(Constants.MaxVelocity.times(controller.getLeftX()*0.6))
      .withVelocityY(Constants.MaxVelocity.times(controller.getLeftY()*0.6))
      .withRotationalRate(Constants.MaxOmega.times(controller.getRightX()* 0.8))));
    else drivetrain.setDefaultCommand(drivetrain.drive(() -> driveReq
    .withVelocityX(Constants.MaxVelocity.times(controller.getRawAxis(1)*0.6))
    .withVelocityY(Constants.MaxVelocity.times(controller.getRawAxis(0)*0.6))
    .withRotationalRate(Constants.MaxOmega.times(controller.getRawAxis(2)* 0.8))));
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Auto.getAuto();
  }
}
