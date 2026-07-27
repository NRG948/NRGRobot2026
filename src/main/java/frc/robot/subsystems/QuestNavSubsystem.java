/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();

  private Pose2d pose = new Pose2d();

  // Offset from robot center to the Quest headset
  // Example: Quest is 0.3m forward, 0.0m left, 0.5m up from robot center
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
  private static final Transform3d QUEST_TO_ROBOT = ROBOT_TO_QUEST.inverse();

  @Override
  public void periodic() {
    System.out.println("\nquestnav periodic method");
    questNav.commandPeriodic();
    int i = 0;

    for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
      if (frame.isTracking()) {
        Pose3d robotPose = frame.questPose3d(); // .transformBy(QUEST_TO_ROBOT);
        pose = robotPose.toPose2d();
        System.out.printf(
            "%.2f %.2f %.1f\n", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        // Feed to your pose estimator:
        // driveSubsystem.addVisionMeasurement(
        //     robotPose.toPose2d(), frame.dataTimestamp(), stdDevs);
      }
    }
  }

  public Pose2d getPose() {
    return pose;
  }
}
