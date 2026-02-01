/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.dashboard.annotations.DashboardDefinition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotPreferences;

/** Helper methods related to the 2026 FRC Rebuilt field. */
@DashboardDefinition
public final class FieldUtils {

  private static AprilTagFieldLayout FIELD_LAYOUT =
      RobotPreferences.FIELD_LAYOUT_PREFERENCE.getValue().loadAprilTagFieldLayout();

  private FieldUtils() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /** Returns true if we are on the Red alliance. Defaults to Blue if alliance is not set. */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red;
  }

  /** Returns the {@link AprilTagFieldLayout} for the current competition year. */
  public static AprilTagFieldLayout getFieldLayout() {
    return FIELD_LAYOUT;
  }

  /** Returns the {@link Pose3d} of the specified April Tag ID. */
  public static Pose3d getAprilTagPose3d(int tagId) {
    return FIELD_LAYOUT.getTagPose(tagId).get();
  }

  /** Returns the {@link Pose2d} of the specified April Tag ID. */
  public static Pose2d getAprilTagPose2d(int tagId) {
    return getAprilTagPose3d(tagId).toPose2d();
  }
}
