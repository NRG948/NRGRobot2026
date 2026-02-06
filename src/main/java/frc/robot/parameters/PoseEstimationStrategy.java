/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public enum PoseEstimationStrategy {
  AverageBestTargets(PoseStrategy.AVERAGE_BEST_TARGETS),
  ClosestToCameraHeight(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT),
  LowestAmbiguity(PoseStrategy.LOWEST_AMBIGUITY),
  MultiTagPnpOnCoprocessor(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
  PnpDistanceTrigSolve(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

  private final PoseStrategy strategy;

  private PoseEstimationStrategy(PoseStrategy strategy) {
    this.strategy = strategy;
  }

  public PoseStrategy getStrategy() {
    return strategy;
  }
}
