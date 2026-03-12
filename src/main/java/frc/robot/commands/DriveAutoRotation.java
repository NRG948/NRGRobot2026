/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.RobotPreferences.ROTATION_PID_CONTROLLER;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveAutoRotation extends DriveUsingController {

  private static final double IZONE_SCALE_FACTOR = 1.5;

  public DriveAutoRotation(Swerve drivetrain, CommandXboxController xboxController) {
    super(drivetrain, xboxController);
    ROTATION_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROTATION_PID_CONTROLLER.reset();
    ROTATION_PID_CONTROLLER.setIZone(drivetrain.getHubAlignmentTolerance() * IZONE_SCALE_FACTOR);
  }

  @Override
  protected double calculateRotationSpeed() {
    double currentOrientation = drivetrain.getOrientation().getRadians();
    double targetOrientation = drivetrain.getAngleToHub();

    double rSpeed = ROTATION_PID_CONTROLLER.calculate(currentOrientation, targetOrientation);
    return rSpeed;
  }
}
