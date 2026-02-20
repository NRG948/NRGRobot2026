/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveAutoOrientToAlliance extends DriveUsingController {

  public DriveAutoOrientToAlliance(Swerve drivetrain, CommandXboxController xboxController) {
    super(drivetrain, xboxController);
  }

  // PID
  @DashboardPIDController(title = "Auto Rotation PID", column = 6, row = 0, width = 2, height = 3)
  private final ProfiledPIDControllerPreference rotationPIDController =
      new ProfiledPIDControllerPreference(
          "Swerve", "Rotation PID Controller", 1, 0, 0, Swerve.getRotationalConstraints());

  private double targetOrientation = -Math.PI;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  protected double calculateRotationSpeed() {
    return calculateRotationSpeed(rotationPIDController);
  }

  private double calculateRotationSpeed(ProfiledPIDControllerPreference controller) {

    double currentOrientation = drivetrain.getOrientation().getRadians();

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    // TODO: Find alternative to getSetpoint() for PID preference
    double rSpeed = feedback; // feedback + (controller.getSetpoint().velocity /
    // Swerve.getRotationalConstraints().maxVelocity);
    return rSpeed;
  }
}
