/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardLayout;
import com.nrg948.dashboard.annotations.DashboardNumberSlider;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardToggleSwitch;
import com.nrg948.preferences.BooleanPreference;
import com.nrg948.preferences.DoublePreference;
import com.nrg948.preferences.EnumPreference;
import com.nrg948.preferences.ProfiledPIDControllerPreference;
import frc.robot.parameters.AprilTagFieldParameters;
import frc.robot.parameters.PoseEstimationStrategy;
import frc.robot.subsystems.Swerve;

/** Defines robot preferences that can be adjusted via the dashboard. */
@DashboardDefinition
public final class RobotPreferences {

  /** A class to group the AprilTag camera enablement preferences. */
  @DashboardDefinition
  public static final class AprilTagPreferences {
    /** Selects whether to enable the front left camera. */
    @DashboardToggleSwitch(title = "Enable Front Left")
    public final BooleanPreference ENABLE_FRONT_LEFT =
        new BooleanPreference("AprilTag", "Enable Front Left", false);

    /** Selects whether to enable the front right camera. */
    @DashboardToggleSwitch(title = "Enable Front Right")
    public final BooleanPreference ENABLE_FRONT_RIGHT =
        new BooleanPreference("AprilTag", "Enable Front Right", false);

    /** Selects whether to enable the back left camera. */
    @DashboardToggleSwitch(title = "Enable Back Left")
    public final BooleanPreference ENABLE_BACK_LEFT =
        new BooleanPreference("AprilTag", "Enable Back Left", false);

    /** Selects whether to enable the back right camera. */
    @DashboardToggleSwitch(title = "Enable Back Right")
    public final BooleanPreference ENABLE_BACK_RIGHT =
        new BooleanPreference("AprilTag", "Enable Back Right", false);
  }

  /** Groups the AprilTag enablement preferences. */
  @DashboardLayout(title = "April Tag", column = 5, row = 0, width = 2, height = 3)
  public static final AprilTagPreferences APRIL_TAG = new AprilTagPreferences();

  /** Selects the field layout. */
  @DashboardComboBoxChooser(title = "Field Layout", column = 7, row = 0, width = 2, height = 1)
  public static EnumPreference<AprilTagFieldParameters> FIELD_LAYOUT_PREFERENCE =
      new EnumPreference<AprilTagFieldParameters>(
          "AprilTag", "Field Layout", AprilTagFieldParameters.k2026RebuiltWelded);

  /** Selects the AprilTag pose estimation strategy. */
  @DashboardComboBoxChooser(
      title = "Pose Est. Strategy",
      column = 7,
      row = 1,
      width = 2,
      height = 1)
  public static EnumPreference<PoseEstimationStrategy> POSE_ESTIMATION_STRATEGY =
      new EnumPreference<PoseEstimationStrategy>(
          "AprilTag", "Pose Est. Strategy", PoseEstimationStrategy.MultiTagPnpOnCoprocessor);

  /**
   * Selects whether to override the Swerve odemetry with high-confidence estimated poses from
   * vision.
   */
  @DashboardToggleSwitch(
      title = "Auto Override Odometry With Pose",
      column = 7,
      row = 2,
      width = 2,
      height = 1)
  public static final BooleanPreference SHOULD_UPDATE_ODOMETRY =
      new BooleanPreference("AprilTag", "Should Update Odometry", true);

  /** Selects the type of robot. */
  @DashboardComboBoxChooser(title = "Robot Selector", column = 0, row = 0, width = 2, height = 1)
  public static final EnumPreference<RobotSelector> ROBOT_TYPE =
      new EnumPreference<>("Robot", "Robot Type", RobotSelector.CompetitionRobot2026);

  /** Selects the auto-rotation PID controller gains. */
  @DashboardPIDController(title = "Auto Rotation PID", column = 3, row = 1, width = 2, height = 3)
  public static final ProfiledPIDControllerPreference ROTATION_PID_CONTROLLER =
      new ProfiledPIDControllerPreference(
          "Swerve", "Rotation PID Controller", 1, 0, 0, Swerve.getRotationalConstraints());

  /** Selects whether rumble functionality is enabled on the driver's controller. */
  @DashboardToggleSwitch(title = "Enable Rumble", column = 2, row = 0, width = 1, height = 1)
  public static final BooleanPreference ENABLE_RUMBLE =
      new BooleanPreference("Drive", "Enable Rumble", true);

  /** Adjusts the sensitivity of the right trigger on the driver's controller. */
  @DashboardNumberSlider(
      title = "Right Trigger Scalar",
      column = 3,
      row = 0,
      width = 2,
      height = 1,
      min = 0,
      max = 1)
  public static final DoublePreference RIGHT_TRIGGER_SCALAR =
      new DoublePreference("Drive", "Right Trigger Scalar", 0.25);

  /** Creates a new instance of RobotPreferences. */
  public RobotPreferences() {}
}
