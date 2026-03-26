/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCalculator {

  public static class ShotSolution {
    public final boolean inRange;
    public final boolean ready;
    public final Rotation2d driveAngle;
    public final double flywheelSpeed;
    public final double effectiveDistance;
    public final double timeOfFlight;

    public ShotSolution(
        boolean inRange,
        boolean ready,
        Rotation2d driveAngle,
        double flywheelSpeed,
        double effectiveDistance,
        double timeOfFlight) {
      this.inRange = inRange;
      this.ready = ready;
      this.driveAngle = driveAngle;
      this.flywheelSpeed = flywheelSpeed;
      this.effectiveDistance = effectiveDistance;
      this.timeOfFlight = timeOfFlight;
    }
  }

  // Configuration
  private Transform2d robotToLauncher = new Transform2d(new Translation2d(), new Rotation2d());
  private double totalLatency = 0.06;
  private int maxAimIterations = 5;
  private double convergenceThresholdMeters = 0.003;
  private double minDistanceMeters = 1.0;
  private double maxDistanceMeters = 5.60;

  private double driveAngleTolerance = 0.06;
  private double flywheelToleranceFrac = 0.03;

  private final InterpolatingDoubleTreeMap flywheelMap;
  private final InterpolatingDoubleTreeMap tofMap;

  public ShotCalculator(InterpolatingDoubleTreeMap flywheelMap) {
    this.flywheelMap = flywheelMap;

    // 5687's Time of Flight LUT (distance_m -> seconds)
    this.tofMap = new InterpolatingDoubleTreeMap();
    tofMap.put(1.52, 1.2);
    tofMap.put(2.13, 1.2);
    tofMap.put(3.048, 1.2);
    tofMap.put(3.66, 1.2);
    tofMap.put(4.2672, 1.25);
    tofMap.put(4.88, 1.3);
    tofMap.put(5.49, 1.5);
  }

  /**
   * Calculates the shot solution.
   *
   * @param currentPose The current estimated pose of the robot on the field.
   * @param currentSpeeds The current robot-relative chassis speeds.
   * @param target The field-relative translation (XY) of the target (e.g. Hub).
   * @param currentFlywheelVelocity The current flywheel velocity (in whatever units the flywheelMap
   *     returns).
   * @return A ShotSolution containing the required robot heading, flywheel speed, etc.
   */
  public ShotSolution calculate(
      Pose2d currentPose,
      ChassisSpeeds currentSpeeds,
      Translation2d target,
      double currentFlywheelVelocity) {

    Pose2d futurePose =
        new Pose2d(
            currentPose.getX() + currentSpeeds.vxMetersPerSecond * totalLatency,
            currentPose.getY() + currentSpeeds.vyMetersPerSecond * totalLatency,
            currentPose
                .getRotation()
                .plus(Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * totalLatency)));

    Pose2d launcherPose = futurePose.transformBy(robotToLauncher);
    Translation2d launcherXY = launcherPose.getTranslation();

    Rotation2d heading = futurePose.getRotation();
    double cos_h = heading.getCos();
    double sin_h = heading.getSin();

    double rvx = currentSpeeds.vxMetersPerSecond;
    double rvy = currentSpeeds.vyMetersPerSecond;
    double omega = currentSpeeds.omegaRadiansPerSecond;

    double fieldVx = rvx * cos_h - rvy * sin_h;
    double fieldVy = rvx * sin_h + rvy * cos_h;

    double offsetX_robot = robotToLauncher.getX();
    double offsetY_robot = robotToLauncher.getY();
    double offsetX_field = offsetX_robot * cos_h - offsetY_robot * sin_h;
    double offsetY_field = offsetX_robot * sin_h + offsetY_robot * cos_h;

    fieldVx += -omega * offsetY_field;
    fieldVy += omega * offsetX_field;

    double lx = launcherXY.getX();
    double ly = launcherXY.getY();
    double tx = target.getX();
    double ty = target.getY();

    double lookaheadX = lx;
    double lookaheadY = ly;
    double effDist = Math.hypot(tx - lookaheadX, ty - lookaheadY);
    double tof = tofMap.get(effDist);

    for (int i = 0; i < maxAimIterations; ++i) {
      double newLookaheadX = lx + fieldVx * tof;
      double newLookaheadY = ly + fieldVy * tof;
      double newDist = Math.hypot(tx - newLookaheadX, ty - newLookaheadY);

      lookaheadX = newLookaheadX;
      lookaheadY = newLookaheadY;

      if (Math.abs(newDist - effDist) < convergenceThresholdMeters) {
        effDist = newDist;
        break;
      }
      effDist = newDist;
      tof = tofMap.get(effDist);
    }

    double flywheelSpeed = flywheelMap.get(effDist);
    tof = tofMap.get(effDist);

    double aimX = tx - lx - fieldVx * tof;
    double aimY = ty - ly - fieldVy * tof;
    Rotation2d aimToTarget = new Rotation2d(aimX, aimY);
    Rotation2d driveAngle = aimToTarget.minus(robotToLauncher.getRotation());

    boolean driveOK =
        Math.abs(currentPose.getRotation().minus(driveAngle).getRadians()) < driveAngleTolerance;

    boolean flywheelOK =
        flywheelSpeed > 0.0
            && Math.abs(currentFlywheelVelocity - flywheelSpeed) / Math.max(flywheelSpeed, 0.1)
                < flywheelToleranceFrac;

    boolean inRange = effDist >= minDistanceMeters && effDist <= maxDistanceMeters;

    return new ShotSolution(
        inRange, inRange && driveOK && flywheelOK, driveAngle, flywheelSpeed, effDist, tof);
  }
}
