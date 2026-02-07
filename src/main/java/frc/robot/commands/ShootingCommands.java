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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;

public final class ShootingCommands {

  // TODO: Implement method that fetches goal velocity for indexer.
  public static Command shoot(Subsystems subsystem) {
    Indexer indexer = subsystem.indexer;
    return Commands.runOnce(() -> indexer.setShootingVelocity(), indexer);
  }

  // For testing shooter speeds. After interpolation table is done and implemented, remove the
  // methods below along with its button bindings in RobotContainer.java.

  public static Command setShooterVelocityToSeven(Subsystems subsystems) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalVelocity(7.0), shooter);
  }

  public static Command decreaseShooterVelocityByPointTwo(Subsystems subsystems) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.addGoalVelocity(-0.2), shooter);
  }

  public static Command increaseShooterVelocityByPointTwo(Subsystems subsystems) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.addGoalVelocity(0.2), shooter);
  }
}
