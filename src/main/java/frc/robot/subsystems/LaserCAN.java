/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class LaserCAN extends SubsystemBase {
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

  /** Creates a new LaserCAN. */
  public LaserCAN(String LaserCANName, double distanceCorrection) {
    this.laserCANName = laserCANName;
    this.distanceCorrection = distanceCorrection;
    logDistance = new DoubleLogEntry(LOG, "/LaserCAN/" + LaserCANName + "/Distance");

    try {
      laserCAN = createLaserCAN(CAN.LASER_CAN_ID, TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    updateTelemetry();
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
  private void updateTelemetry() {
    distance = getDistance();

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
