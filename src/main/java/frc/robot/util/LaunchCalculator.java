/*
 * Copyright (c) 2025-2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.Constants.RobotConstants.PERIODIC_INTERVAL;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Swerve;

public class LaunchCalculator {

  private static final double minDistance = 0.9;
  private static final double maxDistance = 4.9;
  private static final double PHASE_DELAY = 0.03;
  private static final Translation2d robotToLauncherTranslate = new Translation2d(-0.17, 0);
  private static final Transform2d robotToLauncher =
      new Transform2d(robotToLauncherTranslate, Rotation2d.kZero);

  private final Swerve drivetrain;

  public LaunchCalculator(Swerve drivetrain) {
    this.drivetrain = drivetrain;
  }

  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / PERIODIC_INTERVAL));

  private Rotation2d lastDriveAngle;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  // Launching Maps
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    flywheelSpeedMap.put(0.96, 150.0);
    flywheelSpeedMap.put(1.16, 155.0);
    flywheelSpeedMap.put(1.58, 160.0);
    flywheelSpeedMap.put(2.07, 165.0);
    flywheelSpeedMap.put(2.37, 170.0);
    flywheelSpeedMap.put(2.47, 170.0);
    flywheelSpeedMap.put(2.70, 170.0);
    flywheelSpeedMap.put(2.94, 175.0);
    flywheelSpeedMap.put(3.48, 175.0);
    flywheelSpeedMap.put(3.92, 180.0);
    flywheelSpeedMap.put(4.35, 185.0);
    flywheelSpeedMap.put(4.84, 190.0);

    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    // Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getChassisSpeeds();
    Pose2d futureEstimatedPose =
        drivetrain
            .getPosition()
            .exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY,
                    robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY,
                    robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY));

    // Calculate target
    Translation2d target = FieldUtils.getHubLocation();
    Pose2d launcherPosition = futureEstimatedPose.transformBy(robotToLauncher);
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    ChassisSpeeds robotVelocity = robotRelativeVelocity; // drivetrain.getFieldSetpointVelocity();
    Rotation2d robotAngle = drivetrain.getOrientation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : transformVelocity(robotVelocity, robotToLauncherTranslate, robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    boolean isValid =
        lookaheadLauncherToTargetDistance >= minDistance
            && lookaheadLauncherToTargetDistance <= maxDistance;
    // Account for launcher being off center
    Pose2d lookaheadRobotPose = lookaheadPose.transformBy(robotToLauncher.inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / PERIODIC_INTERVAL);
    lastDriveAngle = driveAngle;

    double flywheelVelocity = flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    latestParameters =
        new LaunchingParameters(
            isValid,
            driveAngle,
            driveVelocity,
            flywheelVelocity,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight);

    // Log calculated values
    // Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    // Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    // Logger.recordOutput(
    //     "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    return latestParameters;
  }

  public double getStaticTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  /**
   * Transforms a velocity along a translation.
   *
   * @param velocity The original velocity
   * @param transform The transform to the new position
   * @param currentRotation The current rotation of the robot
   * @return The new velocity
   */
  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getY() * currentRotation.getCos()
                    - transform.getX() * currentRotation.getSin()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    // Rotation2d hubAngle =
    //     new Rotation2d(
    //         Math.asin(
    //             MathUtil.clamp(
    //                 robotToLauncher.getY()
    //                     / target.getDistance(robotPose.getTranslation()),
    //                 -1.0,
    //                 1.0)));
    Rotation2d driveAngle = fieldToHubAngle; // .plus(hubAngle);
    return driveAngle;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
    Translation2d target = FieldUtils.getHubLocation();

    // return new Pose2d(
    //     robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
    return Pose2d.kZero;
  }
}
