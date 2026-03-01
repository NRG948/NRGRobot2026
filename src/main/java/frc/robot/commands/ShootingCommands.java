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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public final class ShootingCommands {

  public static Command shootWhenInRange(Subsystems subsystems) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Swerve drivetrain = subsystems.drivetrain;
    Intake intake = subsystems.intake;
    return Commands.sequence(
        Commands.idle(indexer, shooter, intake, hopper)
            .until(() -> drivetrain.getDistanceToHub() <= Shooter.MAXIMUM_SHOOTING_RANGE),
        shoot(subsystems));
  }

  public static Command shoot(Subsystems subsystems) {
    Swerve drivetrain = subsystems.drivetrain;
    return shootForDistance(subsystems, drivetrain::getDistanceToHub);
  }

  public static Command shootFromHub(Subsystems subsystems) {
    return shootForDistance(subsystems, () -> Shooter.HUB_SHOT_DISTANCE);
  }

  public static Command shootFromTower(Subsystems subsystems) {
    return shootForDistance(subsystems, () -> Shooter.TOWER_SHOT_DISTANCE);
  }

  private static Command shootForDistance(Subsystems subsystems, DoubleSupplier distance) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalDistance(distance.getAsDouble()), shooter),
            feedBallsToShooter(subsystems))
        .finallyDo(
            () -> {
              shooter.disable();
              indexer.disable();
              intake.disable();
              hopper.disable();
            });
  }

  public static Command rampUpShooter(Subsystems subsystems) {
    Shooter shooter = subsystems.shooter;
    Swerve drivetrain = subsystems.drivetrain;
    return Commands.run(() -> shooter.setGoalDistance(drivetrain.getDistanceToHub()), shooter)
        .finallyDo(shooter::disable);
  }

  public static Command feedBallsToShooter(Subsystems subsystems) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;
    Swerve drivetrain = subsystems.drivetrain;

    return Commands.sequence(
        Commands.idle(indexer).until(() -> shooter.atOrNearGoal() && drivetrain.isAlignedToHub()),
        Commands.runOnce(hopper::feed, hopper),
        Commands.runOnce(indexer::feed, indexer),
        Commands.runOnce(intake::intake, intake),
        IntakeCommands.agitateArm(subsystems),
        Commands.idle(intake, indexer));
  }

  public static Command shoot(Subsystems subsystems, double velocity) {
    Indexer indexer = subsystems.indexer;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalVelocity(velocity), shooter),
            feedBallsToShooter(subsystems))
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
