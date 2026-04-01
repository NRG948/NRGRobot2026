/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public final class IndexerCommands {
  public static Command feed(Subsystems subsystems) {
    Rollers indexer = subsystems.indexer;
    Rollers hopper = subsystems.hopper;
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  indexer.feed();
                  hopper.feed();
                },
                indexer,
                hopper),
            Commands.idle(indexer, hopper))
        .finallyDo(
            () -> {
              hopper.disable();
              indexer.disable();
            })
        .withName("Feed");
  }
}
