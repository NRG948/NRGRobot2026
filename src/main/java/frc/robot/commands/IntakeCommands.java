/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Subsystems;

/** A utility class for controlling the intake. */
public final class IntakeCommands {

  /**
   * Returns Command that stows the intake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command stowIntake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    // TODO Flesh out full sequence when other subsystems are finished.
    return Commands.runOnce(intake::disable, intake);
  }

  /**
   * Returns Command that starts the intake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command intake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    // TODO Flesh out full sequence when other subsystems are finished.
    return Commands.runOnce(intake::intake, intake);
  }

  /**
   * Returns Command that starts the outtake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command outtake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    // TODO Flesh out full sequence when other subsystems are finished.
    return Commands.runOnce(intake::outtake, intake);
  }
}
