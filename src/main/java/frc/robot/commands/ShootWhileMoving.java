/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.RobotPreferences.ROTATION_PID_CONTROLLER;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.FuelLaunchSolver;
import frc.robot.util.FuelLaunchSolver.ShootingSolution;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class ShootWhileMoving extends DriveUsingController {

  private final Shooter shooter;
  private final FuelLaunchSolver launchSolver;

  public ShootWhileMoving(Subsystems subsystems, CommandXboxController xboxController) {
    super(subsystems.drivetrain, xboxController);
    this.shooter = subsystems.shooter;
    this.launchSolver = subsystems.getFuelLaunchSolver();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROTATION_PID_CONTROLLER.reset();
  }

  @Override
  protected double calculateRotationSpeed() {
    ShootingSolution solution = launchSolver.getMovingShootingSolution();

    shooter.setGoalVelocity(solution.shooterSpeed());

    double rSpeed =
        ROTATION_PID_CONTROLLER.calculate(
                drivetrain.getOrientation().getRadians(), solution.robotOrientation())
            * RobotPreferences.SOTF_SCALAR.getValue();
    return rSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.disable();
  }
}
