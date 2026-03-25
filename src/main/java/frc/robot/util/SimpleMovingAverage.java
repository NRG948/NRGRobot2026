/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public class SimpleMovingAverage {
  private final double[] buffer;
  private final int windowSize;
  private int index = 0;
  private double sum = 0.0;
  private int count = 0;

  public SimpleMovingAverage(int windowSize) {
    this.windowSize = windowSize;
    this.buffer = new double[windowSize];
  }

  public void add(double value) {
    sum -= buffer[index];
    buffer[index] = value;
    sum += value;
    index = (index + 1) % windowSize;
    if (count < windowSize) {
      count++;
    }
  }

  public double getAverage() {
    if (count == 0) {
      return 0.0;
    }
    return sum / count;
  }

  public void reset() {
    for (int i = 0; i < windowSize; i++) {
      buffer[i] = 0.0;
    }
    index = 0;
    sum = 0.0;
    count = 0;
  }
}
