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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

public final class ShootingCommands {

  public static Command shoot(Subsystems subsystem) {
    Indexer indexer = subsystem.indexer;
    Shooter shooter = subsystem.shooter;
    Swerve drivetrain = subsystem.drivetrain;
    Intake intake = subsystem.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalDistance(drivetrain.getDistanceToHub()), shooter),
            Commands.sequence(
                Commands.idle(indexer).until(shooter::atOrNearGoal),
                Commands.runOnce(indexer::feed, indexer),
                Commands.runOnce(intake::intake, intake),
                Commands.idle(intake, indexer)))
        .finallyDo(
            () -> {
              shooter.disable();
              indexer.disable();
              intake.disable();
            });
  }

  public static Command shoot(Subsystems subsystem, double velocity) {
    Indexer indexer = subsystem.indexer;
    Shooter shooter = subsystem.shooter;
    Intake intake = subsystem.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalVelocity(velocity), shooter),
            Commands.sequence(
                Commands.idle(indexer).until(shooter::atOrNearGoal),
                Commands.runOnce(indexer::feed, indexer),
                Commands.runOnce(intake::intake, intake),
                Commands.idle(intake, indexer)))
        .finallyDo(
            () -> {
              shooter.disable();
              indexer.disable();
              intake.disable();
            });
  }

  public static Command setShooterVelocity(Subsystems subsystems, double velocity) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalVelocity(velocity), shooter);
  }

  public static Command addShooterVelocity(Subsystems subsystems, double increment) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.addGoalVelocity(increment), shooter);
  }
}
