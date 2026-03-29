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
import frc.robot.util.MatchUtil;
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
            .until(() -> drivetrain.getDistanceToTarget() <= Shooter.MAX_SHOOTING_DISTANCE),
        shoot(subsystems));
  }

  public static Command shootWhenInRangeAndOnShift(Subsystems subsystems) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Swerve drivetrain = subsystems.drivetrain;
    Intake intake = subsystems.intake;
    return Commands.sequence(
        Commands.idle(indexer, shooter, intake, hopper)
            .until(
                () ->
                    drivetrain.getDistanceToTarget() <= Shooter.MAX_SHOOTING_DISTANCE
                        && MatchUtil.isHubActive()),
        shoot(subsystems));
  }

  public static Command shoot(Subsystems subsystems) {
    Swerve drivetrain = subsystems.drivetrain;
    return shootForDistance(subsystems, drivetrain::getDistanceToTarget, true)
        .onlyIf(subsystems::atLeastOneCameraConnected);
  }

  public static Command shootFromHub(Subsystems subsystems) {
    return shootForDistance(subsystems, () -> Shooter.HUB_SHOT_DISTANCE, false);
  }

  public static Command shootFromTower(Subsystems subsystems) {
    return shootForDistance(subsystems, () -> Shooter.TOWER_SHOT_DISTANCE, false);
  }

  private static Command shootForDistance(
      Subsystems subsystems, DoubleSupplier distance, boolean shouldWaitForHubAlign) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalDistance(distance.getAsDouble()), shooter),
            feedBallsToShooter(subsystems, shouldWaitForHubAlign))
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
        .finallyDo(shooter::disable);
  }

  public static Command rampUpShooter(Subsystems subsystems, double distance) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalDistance(distance), shooter);
  }

  public static Command feedBallsToShooter(Subsystems subsystems, boolean shouldWaitForHubAlign) {
    Indexer indexer = subsystems.indexer;
    Hopper hopper = subsystems.hopper;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;
    Swerve drivetrain = subsystems.drivetrain;

    return Commands.sequence(
        Commands.idle(indexer)
            .until(
                () ->
                    shooter.atOrNearGoal()
                        && (!shouldWaitForHubAlign || drivetrain.isAlignedToHub())),
        Commands.runOnce(hopper::feed, hopper),
        Commands.runOnce(indexer::feed, indexer),
        Commands.runOnce(intake::intake, intake),
        IntakeCommands.agitateArm(subsystems),
        Commands.idle(intake, indexer));
  }

  public static Command pass(Subsystems subsystems, DoubleSupplier velocity) {
    Indexer indexer = subsystems.indexer;
    Shooter shooter = subsystems.shooter;
    Intake intake = subsystems.intake;
    Hopper hopper = subsystems.hopper;

    return Commands.parallel(
            Commands.run(() -> shooter.setGoalVelocity(velocity.getAsDouble()), shooter),
            feedBallsToShooter(subsystems, false))
        .finallyDo(
            () -> {
              shooter.disable();
              indexer.disable();
              intake.disable();
              hopper.disable();
            });
  }

  public static Command setShooterVelocity(Subsystems subsystems, double velocity) {
    Shooter shooter = subsystems.shooter;
    return Commands.runOnce(() -> shooter.setGoalVelocity(velocity), shooter);
  }
}
