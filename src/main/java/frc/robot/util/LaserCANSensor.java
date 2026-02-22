/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/** Creates a LaserCAN sensor. */
public class LaserCANSensor {
  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  public static final double NO_MEASURMENT = 0.0;

  /** Amount to add to the raw distance measurements to get accurate distances. */
  private double distanceCorrection;

  private LaserCan laserCAN;
  private String name;

  private DoubleLogEntry logDistance;

  /**
   * Creates a LaserCAN Sensor.
   *
   * @param CANID ID of the sensor.
   * @param name Name of the sensor.
   * @param distanceCorrection Distance correction of the sensor.
   */
  public LaserCANSensor(int CANID, String name, double distanceCorrection) {
    this.name = name;
    this.distanceCorrection = distanceCorrection;
    logDistance = new DoubleLogEntry(LOG, "/LaserCAN/" + name + "/Distance");

    try {
      laserCAN = createLaserCAN(CANID, TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
      e.printStackTrace();
    }
  }

  /**
   * Creates and configures a {@link LaserCAN} object.
   *
   * @param id The laserCAN CAN id
   * @param timingBudget The timing budget for the laserCAN sensor.
   * @return A configured LaserCAN object.
   * @throws ConfigurationFailedException
   */
  private LaserCan createLaserCAN(int id, LaserCan.TimingBudget timingBudget)
      throws ConfigurationFailedException {
    LaserCan laserCAN = new LaserCan(id);
    laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
    laserCAN.setRegionOfInterest(
        new LaserCan.RegionOfInterest(8, 8, 8, 8)); // Makes detection region a box
    laserCAN.setTimingBudget(timingBudget);
    return laserCAN;
  }

  /** {@return the laserCAN sensor name} */
  public String getName() {
    return name;
  }

  /**
   * {@return the distance measurement in meters} If no measurement is available, this method
   * returns {@link LaserCANSensor#NO_MEASURMENT}.
   */
  public double getDistance() {
    Measurement measurement = laserCAN.getMeasurement();
    double distance = NO_MEASURMENT;

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      distance = (measurement.distance_mm / 1000.0) + distanceCorrection;
    }

    logDistance.append(distance);
    return distance;
  }
}
