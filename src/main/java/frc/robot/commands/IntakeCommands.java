/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Subsystems;

/** A utility class for controlling the intake. */
public final class IntakeCommands {

  private static final double AGITATE_WAIT_TIME = 0.25;

  /**
   * Returns Command that stows the intake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command stowIntake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    IntakeArm intakeArm = subsystems.intakeArm;
    return Commands.parallel(
        Commands.runOnce(intake::disable, intake),
        Commands.runOnce(() -> intakeArm.setGoalAngle(IntakeArm.STOW_ANGLE), intakeArm));
  }

  public static Command setIntakeArmAngle(Subsystems subsystems, double angle) {
    IntakeArm intakeArm = subsystems.intakeArm;
    return Commands.sequence(
            Commands.runOnce(() -> intakeArm.setGoalAngle(angle), intakeArm),
            Commands.idle(intakeArm).until(intakeArm::atGoalAngle))
        .finallyDo(
            () -> {
              if (angle == IntakeArm.STOW_ANGLE || angle == IntakeArm.EXTENDED_ANGLE) {
                intakeArm.disable();
              }
            });
  }

  /**
   * Returns Command that starts the intake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command intake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    return Commands.sequence(Commands.runOnce(intake::intake, intake), Commands.idle(intake))
        .finallyDo(intake::disable);
  }

  public static Command disableIntake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    return Commands.runOnce(intake::disable, intake);
  }

  /**
   * Returns Command that starts the outtake.
   *
   * @param subsystems The subsystem container object.
   */
  public static Command outtake(Subsystems subsystems) {
    Intake intake = subsystems.intake;
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(intake::outtake, intake),
                Commands.runOnce(indexer::outFeed, indexer),
                Commands.runOnce(hopper::outFeed, hopper)),
            Commands.idle(intake, indexer, hopper))
        .finallyDo(
            () -> {
              intake.disable();
              indexer.disable();
              hopper.disable();
            });
  }

  public static Command agitateArm(Subsystems subsystems) {
    return Commands.sequence(
            setIntakeArmAngle(subsystems, IntakeArm.AGITATE_ANGLE),
            Commands.waitSeconds(AGITATE_WAIT_TIME),
            setIntakeArmAngle(subsystems, IntakeArm.EXTENDED_ANGLE),
            Commands.waitSeconds(AGITATE_WAIT_TIME))
        .repeatedly();
  }
}
