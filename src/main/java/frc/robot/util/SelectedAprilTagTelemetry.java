/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;

@DashboardDefinition
public class SelectedAprilTagTelemetry {
  @DashboardTextDisplay(column = 0, row = 0, title = "X")
  public double selectedAprilTagPoseX;

  @DashboardTextDisplay(column = 1, row = 0, title = "Y")
  public double selectedAprilTagPoseY;

  @DashboardTextDisplay(column = 2, row = 0, title = "Z")
  public double selectedAprilTagPoseZ;

  @DashboardTextDisplay(column = 0, row = 1, title = "Roll")
  public double selectedAprilTagRoll;

  @DashboardTextDisplay(column = 1, row = 1, title = "Pitch")
  public double selectedAprilTagPitch;

  @DashboardTextDisplay(column = 2, row = 1, title = "Yaw")
  public double selectedAprilTagYaw;

  @DashboardTextDisplay(column = 0, row = 2, title = "Angle to Target")
  public double angleToSelectedTarget;

  @DashboardTextDisplay(column = 1, row = 2, title = "Distance to Target")
  public double distanceToSelectedTarget;
}
