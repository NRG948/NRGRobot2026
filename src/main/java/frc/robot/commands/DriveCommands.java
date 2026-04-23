/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/** A factory class for creating robot commands related to driving. */
public final class DriveCommands {

  private static final double DRIVE_UNTIL_LEVEL_X_SPEED = -0.5;
  private static final double DRIVE_UNTIL_LEVEL_TIMEOUT = 1.00;

  /**
   * Resets the orientation of the robot.
   *
   * @param subsystems
   * @return A command that resets the orientation of the robot.
   */
  public static Command resetOrientation(Subsystems subsystems) {
    var drivetrain = subsystems.drivetrain;
    return Commands.runOnce(
        () -> drivetrain.resetOrientation(FieldUtils.getInitialOrientation()), drivetrain);
  }

  public static Command hubAimAndXLock(Swerve drivetrain, CommandXboxController driverController) {
    return Commands.sequence(
        new DriveAutoRotation(drivetrain, driverController).until(drivetrain::isAlignedToHub),
        Commands.run(drivetrain::setXLock, drivetrain).until(() -> !drivetrain.isAlignedToHub()));
  }

  /**
   * Returns a command that interrupts all subsystems.
   *
   * @param subsystems The subsystems container.
   * @return A command that interrupts all subsystems.
   */
  public static Command interruptAll(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.disableAll(), subsystems.getAll())
        .withName("InterruptAll");
  }

  public static Command driveUntilLevel(Subsystems subsystems) {
    return driveUntilLevel(subsystems.drivetrain);
  }

  /**
   * {@return a command which drives the robot toward our alliance wall until it reaches level
   * ground}
   *
   * <p>This command can be invoked to try to get the robot into shooting position either if the
   * robot unintentionally stops on the bump or surfs on top of fuel in our alliance zone during
   * auto.
   */
  public static Command driveUntilLevel(Swerve drivetrain) {
    return Commands.sequence(
            Commands.print("Driving Until Level"),
            Commands.run(() -> drivetrain.drive(DRIVE_UNTIL_LEVEL_X_SPEED, 0, 0, true), drivetrain))
        .withTimeout(DRIVE_UNTIL_LEVEL_TIMEOUT)
        .unless(drivetrain::isLevel)
        .until(drivetrain::isLevel)
        .withName("DriveUntilLevel");
  }

  private DriveCommands() {
    throw new UnsupportedOperationException("This is a utility class.");
  }
}
