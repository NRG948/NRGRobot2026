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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class ShootWhileMoving extends DriveUsingController {

  Shooter shooter;
  ShotCalculator shotCalculator;

  public ShootWhileMoving(Subsystems subsystems, CommandXboxController xboxController) {
    super(subsystems.drivetrain, xboxController);
    this.shooter = subsystems.shooter;
    this.shotCalculator = new ShotCalculator(Shooter.SHOOTER_VELOCITIES);
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
    ShotSolution solution =
        shotCalculator.calculate(
            drivetrain.getPosition(),
            drivetrain.getChassisSpeeds(),
            FieldUtils.getHubLocation(),
            shooter.getCurrentVelocity());

    double currentOrientation = drivetrain.getOrientation().getRadians();
    double targetOrientation = solution.driveAngle.getRadians();

    double feedback = controller.calculate(currentOrientation, targetOrientation);

    shooter.setGoalVelocity(solution.flywheelSpeed);

    return feedback;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.disable();
  }
}
