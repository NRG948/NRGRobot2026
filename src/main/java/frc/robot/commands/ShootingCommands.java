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
import java.util.function.DoubleSupplier;

public final class ShootingCommands {

  public static final double MAXIMUM_SHOOTING_RANGE = 3.5;
  public static final double HUB_SHOT_DISTANCE = 1.3;
  public static final double TOWER_SHOT_DISTANCE = 3.05;

  public static Command shootWhenInRange(Subsystems subsystem) {
    Indexer indexer = subsystem.indexer;
    Shooter shooter = subsystem.shooter;
    Swerve drivetrain = subsystem.drivetrain;
    Intake intake = subsystem.intake;
    return Commands.sequence(
        Commands.idle(indexer, shooter, intake)
            .until(() -> drivetrain.getDistanceToHub() <= MAXIMUM_SHOOTING_RANGE),
        shoot(subsystem));
  }

  public static Command shoot(Subsystems subsystem) {
    Swerve drivetrain = subsystem.drivetrain;
    return shootForDistance(subsystem, drivetrain::getDistanceToHub);
  }

  public static Command shootFromHub(Subsystems subsystem) {
    return shootForDistance(subsystem, () -> HUB_SHOT_DISTANCE);
  }

  public static Command shootFromTower(Subsystems subsystem) {
    return shootForDistance(subsystem, () -> TOWER_SHOT_DISTANCE);
  }

  private static Command shootForDistance(Subsystems subsystem, DoubleSupplier distance) {
    Indexer indexer = subsystem.indexer;
    Shooter shooter = subsystem.shooter;
    Intake intake = subsystem.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalDistance(distance.getAsDouble()), shooter),
            feedBallsToShooter(indexer, shooter, intake))
        .finallyDo(
            () -> {
              shooter.disable();
              indexer.disable();
              intake.disable();
            });
  }

  private static Command feedBallsToShooter(Indexer indexer, Shooter shooter, Intake intake) {
    return Commands.sequence(
        Commands.idle(indexer).until(shooter::atOrNearGoal),
        Commands.runOnce(indexer::feed, indexer),
        Commands.runOnce(intake::intake, intake),
        Commands.idle(intake, indexer));
  }

  public static Command shoot(Subsystems subsystem, double velocity) {
    Indexer indexer = subsystem.indexer;
    Shooter shooter = subsystem.shooter;
    Intake intake = subsystem.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalVelocity(velocity), shooter),
            feedBallsToShooter(indexer, shooter, intake))
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
