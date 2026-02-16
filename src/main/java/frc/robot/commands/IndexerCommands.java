/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public final class IndexerCommands {
  public static Command feed(Subsystems subsystems) {
    Indexer indexer = subsystems.indexer;
    return Commands.runOnce(() -> indexer.feed(), indexer).withName("Feed");
  }

  public static Command disableIndexer(Subsystems subsystems) {
    Indexer indexer = subsystems.indexer;
    return Commands.runOnce(() -> indexer.disable(), indexer).withName("Feed");
  }
}
