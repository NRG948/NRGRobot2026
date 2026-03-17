/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.util.MatchUtil.isRedAlliance;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotPreferences;

public final class FieldUtils {
  private static AprilTagFieldLayout FIELD_LAYOUT =
      RobotPreferences.FIELD_LAYOUT_PREFERENCE.getValue().loadAprilTagFieldLayout();

  private static final int RED_HUB_APRILTAG = 10;
  private static final int BLUE_HUB_APRILTAG = 26;

  // distance (in meters) between the hub's middle april tag and the hub's center
  private static final double APRIL_TAG_TO_HUB = Units.inchesToMeters(47 / 2);

  private static final Translation2d RED_HUB_POSITION =
      FIELD_LAYOUT
          .getTagPose(RED_HUB_APRILTAG)
          .get()
          .toPose2d()
          .getTranslation()
          .plus(new Translation2d(-APRIL_TAG_TO_HUB, 0));
  private static final Translation2d BLUE_HUB_POSITION =
      FIELD_LAYOUT
          .getTagPose(BLUE_HUB_APRILTAG)
          .get()
          .toPose2d()
          .getTranslation()
          .plus(new Translation2d(APRIL_TAG_TO_HUB, 0));

  private static final double FIELD_LENGTH = Units.inchesToMeters(651.2);

  private static final double BLUE_ALLIANCE_LINE = Units.inchesToMeters(158.6);
  private static final double RED_ALLIANCE_LINE = FIELD_LENGTH - BLUE_ALLIANCE_LINE;

  /** {@return the {@link AprilTagFieldLayout} for the current competition year} */
  public static AprilTagFieldLayout getFieldLayout() {
    return FIELD_LAYOUT;
  }

  /** {@return the {@link Pose3d} of the specified April Tag ID} */
  public static Pose3d getAprilTagPose3d(int tagId) {
    return FIELD_LAYOUT.getTagPose(tagId).get();
  }

  /** {@return the {@link Pose2d} of the specified April Tag ID} */
  public static Pose2d getAprilTagPose2d(int tagId) {
    return getAprilTagPose3d(tagId).toPose2d();
  }

  /** {@return the {@link Translation2d} of the center of the alliance-side hub} */
  public static Translation2d getHubLocation() {
    return isRedAlliance() ? RED_HUB_POSITION : BLUE_HUB_POSITION;
  }

  /** {@return the initial orientation of the robot at the start of the match} */
  public static Rotation2d getInitialOrientation() {
    return isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
  }

  /** {@return whether the given point is within the current alliance zone} */
  public static boolean isPointInAllianceZone(Translation2d point) {
    return isRedAlliance() ? point.getX() >= RED_ALLIANCE_LINE : point.getX() <= BLUE_ALLIANCE_LINE;
  }
}
