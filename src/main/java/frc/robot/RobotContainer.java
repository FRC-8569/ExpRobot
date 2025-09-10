// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Telemetry;
import frc.robot.Elevator.RealElevator;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.system();
  public RealElevator elevator = RealElevator.system();
  public Auto auto;
  public XboxController joystick = new XboxController(0);
  public Telemetry telemetry = new Telemetry();
  public double SpeedMode = 0.4;
  public SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(Constants.MaxVelocity.times(0.05))
    .withRotationalDeadband(Constants.MaxOmega.times(0.05))
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position)
    .withDesaturateWheelSpeeds(true);

  public RobotContainer() {
    auto = new Auto(drivetrain, elevator);

    drivetrain.setDefaultCommand(drivetrain.drive(() -> driveRequest
      .withVelocityX(Constants.MaxVelocity.times(joystick.getLeftX()*SpeedMode))
      .withVelocityY(Constants.MaxVelocity.times(joystick.getLeftY()*SpeedMode))
      .withRotationalRate(Constants.MaxOmega.times(-joystick.getRightX()))));
    configureBindings();
  }

  private void configureBindings() {
    // new Trigger(() -> joystick.getLeftBumperButton()).and(() -> SpeedMode >= 0)
    //   .onTrue(Commands.runOnce(() -> SpeedMode -= 0.1));
    // new Trigger(() -> joystick.getRightBumperButton()).and(() -> SpeedMode <= 1)
    //   .onTrue(Commands.runOnce(() -> SpeedMode += 0.1));
    new Trigger(() -> joystick.getRawButton(1)).and(() -> SpeedMode >= 0)
      .onTrue(Commands.runOnce(() -> SpeedMode -= 0.1));
    new Trigger(() -> joystick.getRawButton(2)).and(() -> SpeedMode <= 1)
      .onTrue(Commands.runOnce(() -> SpeedMode += 0.1));

    new Trigger(() -> joystick.getStartButton())
      .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
      drivetrain.registerTelemetry(telemetry::telemerize);
  }

  public Command getAutonomousCommand() {
    return auto.getAuto();
  }
}
