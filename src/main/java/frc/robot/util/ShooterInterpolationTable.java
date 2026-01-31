/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.dashboard.annotations.DashboardDefinition;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

@DashboardDefinition
public class ShooterInterpolationTable {

  /*@DashboardGraph(
   title = "Interpolation Table",
   row = 0,
   column = 0,
   duration = 223.065,
   min = 0,
   max = 10.0)
  */
  private InterpolatingDoubleTreeMap table;

  /*
   * Instantiates interpolation table with corresponding keys and values.
   */
  public ShooterInterpolationTable() {
    table = new InterpolatingDoubleTreeMap();

    // Populate the table with each corresponding accurate entry.
    // table.put(distance, power);
  }

  /*
   * Places an entry of key distance and value power to the interpolation table. 
   */
  public void placeEntry(double distance, double power) {
    table.put(distance, power);
  }

  /*
   * Fetches power determined by distance to power interpolation table. 
   */
  public double getPower(double distance) {
    return table.get(distance);
  }

}
