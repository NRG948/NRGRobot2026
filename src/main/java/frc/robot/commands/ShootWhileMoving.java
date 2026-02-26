/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.preferences.ProfiledPIDControllerPreference;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class ShootWhileMoving extends DriveUsingController {

  Shooter shooter;

  public ShootWhileMoving(Subsystems subsystems, CommandXboxController xboxController) {
    super(subsystems.drivetrain, xboxController);
    this.shooter = subsystems.shooter;
    addRequirements(shooter);
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
  protected double calculateRotationSpeed() {
    return calculateRotationSpeed(rotationPIDController);
  }

  private double calculateRotationSpeed(ProfiledPIDControllerPreference controller) {
    Translation2d robotVelocity =
        new Translation2d(
                drivetrain.getChassisSpeeds().vxMetersPerSecond,
                drivetrain.getChassisSpeeds().vyMetersPerSecond)
            .rotateBy(drivetrain.getOrientation().unaryMinus());

    Translation2d shooterVelocityStill =
        new Translation2d(
            shooter.getVelocityFromInterpolationTable(drivetrain.getDistanceToHub()),
            drivetrain.getAngleToHub());

    Translation2d shooterVelocityMoving = shooterVelocityStill.minus(robotVelocity);

    double currentOrientation = drivetrain.getOrientation().getRadians();

    double angleBetweenVector =
        Math.acos(
            shooterVelocityStill.dot(shooterVelocityMoving)
                / shooterVelocityStill.getNorm()
                / shooterVelocityMoving.getNorm());
    double targetOrientation =
        drivetrain.getAngleToHub()
            + angleBetweenVector * Math.signum(shooterVelocityStill.cross(shooterVelocityMoving));

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    shooter.setGoalVelocity(shooterVelocityMoving.getNorm());

    // TODO: Find alternative to getSetpoint() for PID preference
    double rSpeed = feedback; // feedback + (controller.getSetpoint().velocity /
    // Swerve.getRotationalConstraints().maxVelocity);
    return rSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.disable();
  }
}
