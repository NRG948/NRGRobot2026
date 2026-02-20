/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class AutoRotation extends Command {

  Swerve drivetrain;
  double currentOrientation;
  double targetOrientation;

  public AutoRotation(Swerve drivetrain) {
    this.drivetrain = drivetrain;
  }

  // PID
  @DashboardPIDController(title = "Auto Rotation PID", column = 6, row = 0, width = 2, height = 3)
  private final ProfiledPIDControllerPreference rotationPIDController =
      new ProfiledPIDControllerPreference(
          "Swerve", "Rotation PID Controller", 1, 0, 0, Swerve.getRotationalConstraints());

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    drivetrain.drive(0, 0, calculateRotationSpeed(rotationPIDController), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateRotationSpeed(ProfiledPIDControllerPreference controller) {

    currentOrientation = drivetrain.getOrientation().getRadians();

    targetOrientation = drivetrain.getAngleToHub();

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    // TODO: Find alternative to getSetpoint() for PID preference
    double rSpeed = feedback; // feedback + (controller.getSetpoint().velocity /
    // Swerve.getRotationalConstraints().maxVelocity);
    return rSpeed;
  }
}
