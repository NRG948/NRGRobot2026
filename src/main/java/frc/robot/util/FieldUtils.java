/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.preferences.EnumPreference;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.parameters.AprilTagFieldParameters;

public final class FieldUtils {

  public static EnumPreference<AprilTagFieldParameters> FIELD_LAYOUT_PREFERENCE =
      new EnumPreference<AprilTagFieldParameters>(
          "AprilTag", "Field Layout", AprilTagFieldParameters.k2026RebuiltWelded);

  private static AprilTagFieldLayout FIELD_LAYOUT =
      FIELD_LAYOUT_PREFERENCE.getValue().loadAprilTagFieldLayout();

  private static final int RED_HUB_APRILTAG = 10;
  private static final int BLUE_HUB_APRILTAG = 26;
  private static final Translation2d HUB_POSITION;

  static {
    HUB_POSITION = getHubAprilTagPosition();
  }

  // distance (in meters) between the hub's middle april tag and the hub's center
  private static final double APRIL_TAG_TO_HUB = Units.inchesToMeters(47 / 2);

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red;
  }

  /** Returns the {@link Pose3d} of the specified April Tag ID. */
  public static Pose3d getAprilTagPose3d(int tagId) {
    return FIELD_LAYOUT.getTagPose(tagId).get();
  }

  /** Returns the {@link Pose2d} of the specified April Tag ID. */
  public static Pose2d getAprilTagPose2d(int tagId) {
    return getAprilTagPose3d(tagId).toPose2d();
  }

  private static int getHubAprilTagID() {
    if (isRedAlliance()) {
      return RED_HUB_APRILTAG;
    }
    return BLUE_HUB_APRILTAG;
  }

  /**
   * @return Translation2d of allicance-side hub's center April tag
   */
  private static Translation2d getHubAprilTagPosition() {
    return getAprilTagPose2d(getHubAprilTagID()).getTranslation();
  }

  public static Translation2d getHubLocation() {
    if (isRedAlliance()) {
      return getHubAprilTagPosition().minus(new Translation2d(APRIL_TAG_TO_HUB, 0));
    }
    return getHubAprilTagPosition().plus(new Translation2d(APRIL_TAG_TO_HUB, 0));
  }

  public static AprilTagFieldLayout getFieldLayout() {
    return FIELD_LAYOUT;
  }
}
