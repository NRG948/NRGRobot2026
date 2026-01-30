/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCAN extends SubsystemBase {
  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  public static final double NO_MEASURMENT = 0.0;

  /** Amount to add to the raw distance measurements to get accurate distances. */
  private double distanceCorrection;

  private LaserCAN laserCAN;
  private String laserCANName;

  private double distance = NO_MEASURMENT;
  private boolean hasValidMeasurement = false;

  private DoubleLogEntry logDistance;

  /** Creates a new LaserCAN. */
  public LaserCAN(String LaserCANName, double distanceCorrection) {
    this.laserCANName = laserCANName;
    this.distanceCorrection = distanceCorrection;
    logDistance = new DoubleLogEntry(LOG, "/LaserCAN/" + LaserCANName + "/Distance");

    // try {
    //   laserCAN =
    // } catch {

    // }
  }

  private LaserCan createLaserCAN(int id, LaserCan.TimingBudget TimingBudget) {
    return new LaserCan(id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
