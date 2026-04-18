/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public record MotorCurrentConfig(
    double supplyCurrentLimit, double statorCurrentLimit, boolean enableCurrentLimit) {
  public static final double DEFAULT_SUPPLY_CURRENT_LIMIT = 70.0;
  public static final double DEFAULT_STATOR_CURRENT_LIMIT = 120.0;

  public MotorCurrentConfig {
    if (supplyCurrentLimit == 0.0) {
      supplyCurrentLimit = DEFAULT_SUPPLY_CURRENT_LIMIT;
    }
    if (statorCurrentLimit == 0.0) {
      statorCurrentLimit = DEFAULT_STATOR_CURRENT_LIMIT;
    }
  }

  public MotorCurrentConfig() {
    this(DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, true);
  }

  public MotorCurrentConfig(double supplyCurrentLimit, double statorCurrentLimit) {
    this(supplyCurrentLimit, statorCurrentLimit, true);
  }

  public MotorCurrentConfig(boolean enableCurrentLimit) {
    this(DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, enableCurrentLimit);
  }
}
;
