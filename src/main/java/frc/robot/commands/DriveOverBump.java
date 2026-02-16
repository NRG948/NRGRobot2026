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

public class DriveOverBump extends DriveUsingController {
  private static final double X_CENTER_LINE = 8.27;
  private static final double Y_CENTER_LINE = 4.03;

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

  private double calculateRotationSpeed(ProfiledPIDControllerPreference controller) {

    double currentOrientation = drivetrain.getOrientation().getRadians();

    double targetOrientation = getTargetAngle(currentOrientation);

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    // TODO: Find alternative to getSetpoint() for PID preference
    double rSpeed = feedback;
    return rSpeed;
  }

  private double getTargetAngle(double currentOrientation) {
    Translation2d currentPosition = drivetrain.getPosition().getTranslation();

    if (currentPosition.getX() < X_CENTER_LINE) { // On blue alliance side
      if (currentPosition.getY()
          < Y_CENTER_LINE) { // To the right of the field (from blue alliance)
        return (0 + RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      } else { // To the left of the field (from blue alliance)
        return (0 - RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      }
    } else { // On red alliance side
      if (currentPosition.getY() < Y_CENTER_LINE) {
        return (180 - RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      } else {
        return (180 +
         RobotPreferences.DRIVE_OVER_BUMP_ANGLE);
      }
    }
  }
}
