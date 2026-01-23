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
public class EstimatedPoseTelemetry {
  @DashboardTextDisplay(column = 0, row = 0, title = "X")
  public double lastEstimatedPoseX;

  @DashboardTextDisplay(column = 1, row = 0, title = "Y")
  public double lastEstimatedPoseY;

  @DashboardTextDisplay(column = 2, row = 0, title = "Yaw")
  public double lastEstimatedPoseYaw;
}
