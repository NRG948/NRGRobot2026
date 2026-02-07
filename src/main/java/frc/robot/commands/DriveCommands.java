/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/** A utitility class for Drive related commands. */
public final class DriveCommands {
  /**
   * Resets the orientation of the robot.
   *
   * @param drivetrain
   * @return A command that resets the orientation of the robot.
   */
  public static Command resetOrientation(Swerve drivetrain) {
    return Commands.runOnce(
        () -> drivetrain.resetOrientation(FieldUtils.getInitialOrientation()), drivetrain);
  }

  private DriveCommands() {
    throw new UnsupportedOperationException("This is a utility class.");
  }
}
