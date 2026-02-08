/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.RobotPreferences.ROTATION_PID_CONTROLLER;

import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveWithAutoRotation extends DriveUsingController {

  public DriveWithAutoRotation(Swerve drivetrain, CommandXboxController xboxController) {
    super(drivetrain, xboxController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROTATION_PID_CONTROLLER.reset(drivetrain.getOrientation().getRotations(), 0.0);
  }

  @Override
  protected double calculateRotationSpeed() {
    return calculateRotationSpeed(ROTATION_PID_CONTROLLER);
  }

  private double calculateRotationSpeed(ProfiledPIDControllerPreference controller) {

    double currentOrientation = drivetrain.getOrientation().getRadians();

    double targetOrientation = drivetrain.getAngleToHub();

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    // TODO: Find alternative to getSetpoint() for PID preference
    double rSpeed = feedback + controller.getSetpoint().velocity;
    return rSpeed;
  }
}
