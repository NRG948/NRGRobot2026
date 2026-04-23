/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.subsystems.Shooter.HUB_SHOT_DISTANCE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class ShootingCommands {

  public static Command shootWhenInRange(Subsystems subsystems) {
    Rollers indexer = subsystems.indexer;
    Rollers hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Swerve drivetrain = subsystems.drivetrain;
    Intake intake = subsystems.intake;
    return Commands.sequence(
            Commands.idle(indexer, shooter, intake, hopper)
                .until(() -> drivetrain.getDistanceToTarget() <= Shooter.MAX_SHOOTING_DISTANCE),
            shoot(subsystems))
        .withName("ShootWhenInRange");
  }

  public static Command shootWhenInRangeAndOnShift(Subsystems subsystems) {
    Rollers indexer = subsystems.indexer;
    Rollers hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Swerve drivetrain = subsystems.drivetrain;
    Intake intake = subsystems.intake;
    return Commands.sequence(
            Commands.idle(indexer, shooter, intake, hopper)
                .until(
                    () ->
                        drivetrain.getDistanceToTarget() <= Shooter.MAX_SHOOTING_DISTANCE
                            && MatchUtil.isHubActive()),
            shoot(subsystems))
        .withName("ShootWhenInRangeAndOnShift");
  }

  public static Command shoot(Subsystems subsystems) {
    Swerve drivetrain = subsystems.drivetrain;
    Shooter shooter = subsystems.shooter;
    return shootForDistance(
            subsystems,
            drivetrain::getDistanceToTarget,
            () ->
                shooter.atOrNearGoal()
                    && drivetrain.isAlignedToHub()
                    && (MatchUtil.isTeleop() || drivetrain.isLevel()))
        .onlyIf(subsystems::atLeastOneCameraConnected);
  }

  public static Command shootFromHub(Subsystems subsystems) {
    return shootForDistance(
            subsystems, () -> Shooter.HUB_SHOT_DISTANCE, subsystems.shooter::atOrNearGoal)
        .withName("ShootFromHub");
  }

  public static Command shootFromTower(Subsystems subsystems) {
    return shootForDistance(
            subsystems, () -> Shooter.TOWER_SHOT_DISTANCE, subsystems.shooter::atOrNearGoal)
        .withName("ShootFromTower");
  }

  private static Command shootForDistance(
      Subsystems subsystems, DoubleSupplier distance, BooleanSupplier readyToShoot) {
    Rollers indexer = subsystems.indexer;
    Rollers hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalDistance(distance.getAsDouble()), shooter),
            feedBallsToShooter(subsystems, readyToShoot))
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
    return Commands.run(() -> shooter.setGoalDistance(drivetrain.getDistanceToTarget()), shooter)
        .finallyDo(shooter::disable)
        .withName("RampUpShooter");
  }

  public static Command rampUpShooterForHub(Subsystems subsystems) {
    Shooter shooter = subsystems.shooter;

    return Commands.run(() -> shooter.setGoalDistance(HUB_SHOT_DISTANCE), shooter)
        .finallyDo(shooter::disable)
        .withName("RampUpShooterForHub");
  }

  public static Command rampUpShooterForDistance(Subsystems subsystems, double distance) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalDistance(distance), shooter)
        .withName("RampUpShooterForDistance");
  }

  public static Command feedBallsToShooter(Subsystems subsystems, BooleanSupplier readyToShoot) {
    Rollers indexer = subsystems.indexer;
    Rollers hopper = subsystems.hopper;
    Intake intake = subsystems.intake;

    return Commands.sequence(
            Commands.idle(indexer).until(readyToShoot),
            Commands.runOnce(indexer::feed, indexer),
            Commands.runOnce(hopper::feed, hopper),
            Commands.runOnce(intake::intakeWhileShooting, intake),
            IntakeCommands.agitateArm(subsystems),
            Commands.idle(intake, indexer))
        .withName("FeedBallsToShooter");
  }
}
