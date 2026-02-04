/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/** A utility class for Drive related commands. */
public final class DriveCommands {

  /**
   * Resets the orientation of the robot.
   *
   * @param subsystems
   * @return A command that resets the orientation of the robot.
   */
  public static Command resetOrientation(Subsystems subsystems) {
    Swerve drivetrain = subsystems.drivetrain;
    return Commands.runOnce(
        () ->
            drivetrain.resetOrientation(
                FieldUtils.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero),
        drivetrain);
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

  private DriveCommands() {
    throw new UnsupportedOperationException("This is a utility class.");
  }
}
