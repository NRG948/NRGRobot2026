/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardField;
import com.nrg948.dashboard.annotations.DashboardMatchTime;
import com.nrg948.dashboard.model.GameField;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchTime;
import java.util.Optional;

@DashboardDefinition
public final class RobotOperator {
  private final Swerve drivetrain;
  private final Optional<AprilTag> frontLeftCamera;
  private final Optional<AprilTag> frontRightCamera;
  private final Optional<AprilTag> backLeftCamera;
  private final Optional<AprilTag> backRightCamera;

  @DashboardComboBoxChooser(
      title = "Autonomous Routine",
      column = 9,
      row = 2,
      width = 3,
      height = 1)
  private final SendableChooser<Command> autoChooser;

  @DashboardComboBoxChooser(title = "Autonomous Delay", column = 9, row = 3, width = 3, height = 1)
  private final SendableChooser<Integer> delayChooser;

  public RobotOperator(Subsystems subsystems, Autos autonomous) {
    drivetrain = subsystems.drivetrain;
    frontLeftCamera = subsystems.frontLeftCamera;
    frontRightCamera = subsystems.frontRightCamera;
    backLeftCamera = subsystems.backLeftCamera;
    backRightCamera = subsystems.backRightCamera;
    autoChooser = autonomous.getAutoChooser();
    delayChooser = autonomous.getDelayChooser();
  }

  @DashboardField(
      title = "Field",
      row = 0,
      column = 0,
      height = 4,
      width = 7,
      game = GameField.REBUILT)
  private Field2d field = new Field2d();

  @DashboardMatchTime(title = "Match Time", row = 0, column = 9, width = 3, height = 2)
  public static double getMatchTime() {
    return MatchTime.getMatchTime();
  }

  @DashboardBooleanBox(
      title = "Front Left Camera Connected",
      column = 7,
      row = 2,
      width = 1,
      height = 1)
  public boolean frontLeftCameraIsConnected() {
    return frontLeftCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(
      title = "Front Right Camera Connected",
      column = 8,
      row = 2,
      width = 1,
      height = 1)
  public boolean frontRightCameraIsConnected() {
    return frontRightCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(
      title = "Back Left Camera Connected",
      column = 7,
      row = 3,
      width = 1,
      height = 1)
  public boolean backLeftCameraIsConnected() {
    return backLeftCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(
      title = "Back Right Camera Connected",
      column = 8,
      row = 3,
      width = 1,
      height = 1)
  public boolean backRightCameraIsConnected() {
    return backRightCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(title = "Within Range", column = 7, row = 0, width = 2, height = 2)
  public boolean isWithinShootingRange() {
    return drivetrain.getDistanceToHub() <= ShootingCommands.MAXIMUM_SHOOTING_RANGE;
  }

  public void periodic() {
    field.setRobotPose(drivetrain.getPosition());
  }
}
