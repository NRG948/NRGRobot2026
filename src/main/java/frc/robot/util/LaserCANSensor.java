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

/** Creates a LaserCAN sensor. Make sure to call updateTelemetry() before calling getDistance(). */
public class LaserCANSensor {
  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  public static final double NO_MEASURMENT = 0.0;

  /** Amount to add to the raw distance measurements to get accurate distances. */
  private double distanceCorrection;

  private LaserCan laserCAN;
  private String laserCANName;

  private double distance = NO_MEASURMENT;
  private boolean hasValidMeasurement = false;

  private DoubleLogEntry logDistance;

  /**
   * Creates a LaserCAN Sensor.
   *
   * @param CANID ID of the sensor.
   * @param LaserCANName Name of the sensor.
   * @param distanceCorrection Distance correction of the sensor.
   */
  public LaserCANSensor(int CANID, String LaserCANName, double distanceCorrection) {
    this.laserCANName = laserCANName;
    this.distanceCorrection = distanceCorrection;
    logDistance = new DoubleLogEntry(LOG, "/LaserCAN/" + LaserCANName + "/Distance");

    try {
      laserCAN = createLaserCAN(CANID, TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
      e.printStackTrace();
    }
  }

  public boolean hasValidMeasurement() {
    return hasValidMeasurement;
  }

  public double getDistance() {
    return distance;
  }

  private LaserCan createLaserCAN(int id, LaserCan.TimingBudget timingBudget)
      throws ConfigurationFailedException {
    LaserCan laserCAN = new LaserCan(id);
    laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
    laserCAN.setRegionOfInterest(
        new LaserCan.RegionOfInterest(8, 8, 8, 8)); // Makes detection region a box
    laserCAN.setTimingBudget(timingBudget);
    return laserCAN;
  }

  /** Updates and logs the current sensors states. */
  public void updateTelemetry() {
    distance = getDistance(laserCAN);

    if (distance == NO_MEASURMENT) {
      hasValidMeasurement = false;
    } else {
      hasValidMeasurement = true;
      distance += distanceCorrection;
    }

    logDistance.append(distance);
  }

  private double getDistance(LaserCan laserCan) {
    if (laserCan == null) {
      return NO_MEASURMENT;
    }

    Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm / 1000.0;
    } else {
      return NO_MEASURMENT;
    }
  }
}
