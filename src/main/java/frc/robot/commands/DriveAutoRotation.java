/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveAutoRotation extends DriveUsingController {

  private Pose2d hubAprilTag = FieldUtils.getHubAprilTag();
  private Pose2d hubLocation =
      new Pose2d(hubAprilTag.getMeasureX(), hubAprilTag.getMeasureY(), Rotation2d.kZero);

  public DriveAutoRotation(Swerve drivetrain, CommandXboxController xboxController) {
    super(drivetrain, xboxController);
  }

  // PID
  @DashboardPIDController(title = "Auto Rotation PID", column = 6, row = 0, width = 2, height = 3)
  private final ProfiledPIDControllerPreference rotationPIDController =
      new ProfiledPIDControllerPreference(
          "Shooter", "Rotation PID Controller", 1, 0, 0, Swerve.getRotationalConstraints());

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * @return Radians of angle difference from bot to hub
   */
  private double getRotationtoHub() {
    Rotation2d angleDiff =
        drivetrain.getPosition().getTranslation().minus(hubLocation.getTranslation()).getAngle();
    double angleDiffRad = angleDiff.getRadians();
    return angleDiffRad;
  }

  @Override
  private double calculateRotationSpeed(PIDControllerPreference controller) {

    double currentOrientation = drivetrain.getOrientation().getRotations();

    double targetOrientation = getRotationtoHub();

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    double rSpeed =
        feedback
            + (controller.getSetpoint().velocity / Swerve.getRotationalConstraints().maxVelocity);
    return rSpeed;
  }
}
