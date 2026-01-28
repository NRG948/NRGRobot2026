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
import com.nrg948.dashboard.annotations.DashboardToggleSwitch;
import com.nrg948.preferences.BooleanPreference;
import com.nrg948.preferences.DoublePreference;
import com.nrg948.preferences.EnumPreference;

/** Defines robot preferences that can be adjusted via the dashboard. */
@DashboardDefinition
public final class RobotPreferences {

  @DashboardDefinition
  public static final class AprilTagPreferences {
    @DashboardToggleSwitch(title = "Enable Front Left")
    public final BooleanPreference ENABLE_FRONT_LEFT =
        new BooleanPreference("AprilTag", "Enable Front Left", false);

    @DashboardToggleSwitch(title = "Enable Front Right")
    public final BooleanPreference ENABLE_FRONT_RIGHT =
        new BooleanPreference("AprilTag", "Enable Front Right", false);
  }

  @DashboardLayout(title = "April Tag", column = 0, row = 1, width = 1, height = 3)
  public static final AprilTagPreferences APRIL_TAG = new AprilTagPreferences();

  /** Creates a new instance of RobotPreferences. */
  public RobotPreferences() {}

  /** Selects the type of robot. */
  @DashboardComboBoxChooser(title = "Robot Selector", column = 0, row = 0, width = 2, height = 1)
  public static final EnumPreference<RobotSelector> ROBOT_TYPE =
      new EnumPreference<>("Robot", "Robot Type", RobotSelector.CompetitionRobot2026);

  /** Enables or disables rumble functionality on the driver's controller. */
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
}
