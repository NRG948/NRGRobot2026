/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public record MotorCurrentConfig(double supplyCurrentLimit, double statorCurrentLimit) {
  public MotorCurrentConfig {
    if (supplyCurrentLimit == 0.0) {
      supplyCurrentLimit = 70.0;
    }
    if (statorCurrentLimit == 0.0) {
      statorCurrentLimit = 120.0;
    }
  }

  public MotorCurrentConfig() {
    this(70.0, 120.0);
  }
}
;
