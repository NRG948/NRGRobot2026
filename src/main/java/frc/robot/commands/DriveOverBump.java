/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.RobotPreferences.ROTATION_PID_CONTROLLER;

import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

public class DriveOverBump extends DriveUsingController {
  /** Creates a new DriveOverBump. */
  public DriveOverBump(Swerve drivetrain, CommandXboxController xboxController) {
    super(drivetrain, xboxController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROTATION_PID_CONTROLLER.reset(drivetrain.getOrientation().getRotations(), 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  protected double calculateRotationSpeed() {
    return calculateRotationSpeed(ROTATION_PID_CONTROLLER);
  }

  private double calculateRotationSpeed(ProfiledPIDControllerPreference rotationPidController) {

    double currentOrientation = drivetrain.getOrientation().getRadians();

    return 0; // TODO: TEMPORARY--Please change this
  }

  private double getTargetAngle(double currentOrientation) {
    Translation2d currentPosition = drivetrain.getPosition().getTranslation();

    if (FieldUtils.getCurrentZone(currentPosition) == FieldUtils.FieldZones.BLUE_ALLIANCE_ZONE) {
      double angle1 = (180 - RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      double angle2 = (180 + RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      // TODO: base on swerve and orientations
    } else if (FieldUtils.getCurrentZone(currentPosition) == FieldUtils.FieldZones.RED_ALLIANCE_ZONE) {
      double angle1 = -RobotPreferences.DRIVE_OVER_BUMP_ANGLE;
      double angle2 = RobotPreferences.DRIVE_OVER_BUMP_ANGLE;
    } else {

    }

    return 0; // TODO: TEMPORARY--Please change this
  }
}
