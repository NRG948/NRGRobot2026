/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.RobotPreferences.isCompBot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * A utility class which calculates shooting solutions for both the static and moving
 * ("shoot-on-the-fly") cases.
 */
public class FuelLaunchSolver {

  /** Tunable value that compensates for latency in sensor measurement and robot reaction time. */
  private static final double PHASE_DELAY = 0.03;

  private static final int SOLVER_ITERATIONS = 6;

  /** Maps distances (from the robot's center to our hub) into corresponding shooter velocities. */
  private static final InterpolatingDoubleTreeMap SHOOTER_VELOCITIES =
      new InterpolatingDoubleTreeMap();

  /** Maps distances (from the robot's center to our hub) into corresponding fuel time of flight. */
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

  static {
    if (isCompBot()) {
      // Competition bot 70 degree hood
      SHOOTER_VELOCITIES.put(1.28, 13.0);
      SHOOTER_VELOCITIES.put(1.35, 13.25);
      SHOOTER_VELOCITIES.put(1.67, 14.0);
      SHOOTER_VELOCITIES.put(2.0, 15.25);
      SHOOTER_VELOCITIES.put(2.33, 16.25);
      SHOOTER_VELOCITIES.put(2.66, 17.5);
      SHOOTER_VELOCITIES.put(3.05, 19.0);
      SHOOTER_VELOCITIES.put(3.35, 21.5);
      SHOOTER_VELOCITIES.put(3.67, 29.5);
    } else {
      // Practice bot 70 degree hood
      SHOOTER_VELOCITIES.put(1.28, 13.25);
      SHOOTER_VELOCITIES.put(1.35, 13.5);
      SHOOTER_VELOCITIES.put(1.67, 14.25);
      SHOOTER_VELOCITIES.put(2.0, 15.5);
      SHOOTER_VELOCITIES.put(2.33, 16.75);
      SHOOTER_VELOCITIES.put(2.66, 18.0);
      SHOOTER_VELOCITIES.put(3.05, 19.5);
      SHOOTER_VELOCITIES.put(3.35, 22.0);
      SHOOTER_VELOCITIES.put(3.67, 30.0);
    }

    // Pracetice bot 70 degree hood
    TIME_OF_FLIGHT.put(1.28, 28.5 / 30.0);
    TIME_OF_FLIGHT.put(1.35, 29.0 / 30.0);
    TIME_OF_FLIGHT.put(1.67, 31.0 / 30.0);
    TIME_OF_FLIGHT.put(2.0, 35.0 / 30.0);
    TIME_OF_FLIGHT.put(2.33, 39.33 / 30.0);
    TIME_OF_FLIGHT.put(2.66, 41.75 / 30.0);
    TIME_OF_FLIGHT.put(3.05, 43.66 / 30.0);
    TIME_OF_FLIGHT.put(3.35, 46.0 / 30.0);
    TIME_OF_FLIGHT.put(3.67, 48.0 / 30.0);
  }

  public record ShootingSolution(
      boolean isValid,
      double distanceToTarget,
      double timeOfFlight,
      double robotOrientation,
      double shooterSpeed) {}

  private static final ShootingSolution INVALID_SOLUTION = new ShootingSolution(false, 0, 0, 0, 0);

  private final Swerve drivetrain;
  private ShootingSolution staticShootingSolution;
  private ShootingSolution movingShootingSolution;

  public FuelLaunchSolver(Swerve drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void solve() {
    Pose2d currentRobotPose = drivetrain.getPosition();

    // Don't bother calculating shooting solutions if the robot is not in our Alliance Zone.
    if (!FieldUtils.isPointInAllianceZone(currentRobotPose.getTranslation())) {
      staticShootingSolution = INVALID_SOLUTION;
      movingShootingSolution = INVALID_SOLUTION;

      return;
    }

    // Calculate robot velocity relative to the field coordinates.
    ChassisSpeeds robotRelativeVelocity = drivetrain.getChassisSpeeds();
    Translation2d fieldRelativeVelocity =
        new Translation2d(
                robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond)
            .rotateBy(drivetrain.getOrientation().unaryMinus());

    // Calculate estimated robot pose while accounting for phase delay.
    Pose2d predictedRobotPose =
        currentRobotPose.exp(
            new Twist2d(
                fieldRelativeVelocity.getX() * PHASE_DELAY,
                fieldRelativeVelocity.getY() * PHASE_DELAY,
                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY));

    Translation2d hubPosition = FieldUtils.getHubLocation();
    // First calculate the stationary (static) shooter solution.
    double staticDistanceToHub = hubPosition.getDistance(currentRobotPose.getTranslation());
    double staticTimeOfFlight = TIME_OF_FLIGHT.get(staticDistanceToHub);
    double staticShooterSpeed = SHOOTER_VELOCITIES.get(staticDistanceToHub);
    double staticRobotAngle = drivetrain.getAngleToHub();
    boolean isValid =
        staticDistanceToHub >= Shooter.HUB_SHOT_DISTANCE
            && staticDistanceToHub <= Shooter.MAX_SHOT_DISTANCE;

    staticShootingSolution =
        new ShootingSolution(
            isValid, staticDistanceToHub, staticTimeOfFlight, staticRobotAngle, staticShooterSpeed);

    // Calculate how fast the shooter's center position is moving relative to the field.
    // For simplicity, assume it is the same as the robot's velocity and ignore the impact of the
    // robot's rotational velocity on the shooter's velocity.
    Translation2d launcherToFieldVelocity = fieldRelativeVelocity;

    // Setup for shoot-on-the-fly iterative solution.
    double hubX = hubPosition.getX();
    double hubY = hubPosition.getY();
    double predictedRobotX = predictedRobotPose.getX();
    double predictedRobotY = predictedRobotPose.getY();
    double predictedX = 0;
    double predictedY = 0;
    double predictedDistanceToHub = staticDistanceToHub;
    double timeOfFlight = staticTimeOfFlight;

    // Iterate to solve for the velocity imparted by the robot onto the fuel as it is launched.
    for (int i = 0; i < SOLVER_ITERATIONS; i++) {
      predictedX = predictedRobotX + launcherToFieldVelocity.getX() * timeOfFlight;
      predictedY = predictedRobotY + launcherToFieldVelocity.getY() * timeOfFlight;
      predictedDistanceToHub = Math.hypot(hubX - predictedX, hubY - predictedY);
      timeOfFlight = TIME_OF_FLIGHT.get(predictedDistanceToHub);
    }

    double movingShooterSpeed = SHOOTER_VELOCITIES.get(predictedDistanceToHub);
    double movingRobotAngle = Math.atan2(hubY - predictedY, hubX - predictedX);
    isValid = predictedDistanceToHub <= Shooter.MAX_SHOT_DISTANCE;

    movingShootingSolution =
        new ShootingSolution(
            isValid, predictedDistanceToHub, timeOfFlight, movingRobotAngle, movingShooterSpeed);
  }

  public ShootingSolution getStaticShootingSolution() {
    return staticShootingSolution;
  }

  public ShootingSolution getMovingShootingSolution() {
    return movingShootingSolution;
  }
}
