/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.dashboard.annotations.DashboardAlerts;
import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardField;
import com.nrg948.dashboard.annotations.DashboardMatchTime;
import com.nrg948.dashboard.annotations.DashboardSplitButtonChooser;
import com.nrg948.dashboard.model.GameField;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos;
import frc.robot.parameters.AutoSide;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchUtil;
import java.util.Optional;

@DashboardDefinition
public final class RobotOperator {
  private final Swerve drivetrain;
  private final IntakeArm intakeArm;
  private final Optional<AprilTag> frontLeftCamera;
  private final Optional<AprilTag> frontRightCamera;
  private final Optional<AprilTag> backLeftCamera;
  private final Optional<AprilTag> backRightCamera;

  /** Selects whether to use left or right side auto */
  @DashboardSplitButtonChooser(
      title = "Autonomous Start Side",
      column = 9,
      row = 2,
      width = 3,
      height = 1)
  public final SendableChooser<AutoSide> sideChooser = Autos.getSideChooser();

  @DashboardComboBoxChooser(
      title = "Autonomous Routine",
      column = 9,
      row = 3,
      width = 3,
      height = 1)
  private final SendableChooser<Command> autoChooser = Autos.getAutoChooser();

  @DashboardComboBoxChooser(title = "Autonomous Delay", column = 9, row = 4, width = 3, height = 1)
  private final SendableChooser<Integer> delayChooser = Autos.getDelayChooser();

  @DashboardAlerts(title = "Alerts", column = 0, row = 4, width = 3, height = 2)
  private final Alert[] alerts = new Alert[] {Autos.getInvalidAutoAlert()};

  public RobotOperator(Subsystems subsystems) {
    drivetrain = subsystems.drivetrain;
    intakeArm = subsystems.intakeArm;
    frontLeftCamera = subsystems.frontLeftCamera;
    frontRightCamera = subsystems.frontRightCamera;
    backLeftCamera = subsystems.backLeftCamera;
    backRightCamera = subsystems.backRightCamera;
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
    return MatchUtil.getMatchTime();
  }

  @DashboardBooleanBox(title = "Is Hub Active", column = 3, row = 4, width = 2, height = 2)
  public boolean isHubActive() {
    return MatchUtil.isHubActive();
  }

  @DashboardBooleanBox(title = "Is Good Scoring Time", column = 5, row = 4, width = 2, height = 2)
  public boolean isGoodScoringTime() {
    return MatchUtil.isGoodScoringTime();
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

  @DashboardBooleanBox(title = "Within Range", column = 7, row = 0, width = 2, height = 1)
  public boolean isWithinShootingRange() {
    return drivetrain.getDistanceToHub() <= Shooter.MAX_SHOOTING_DISTANCE;
  }

  @DashboardBooleanBox(title = "Aligned to Hub", column = 7, row = 1, width = 2, height = 1)
  public boolean isAlignedToHub() {
    return drivetrain.isAlignedToHub();
  }

  @DashboardCommand(
      title = "Set Extended Position",
      column = 7,
      row = 4,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setExtendedPositionCommand() {
    return Commands.runOnce(() -> intakeArm.setExtendedPosition(), intakeArm)
        .withName("Set Extended Position")
        .ignoringDisable(true);
  }

  @DashboardCommand(
      title = "Set Stowed Position",
      column = 7,
      row = 5,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setStowedPositionCommand() {
    return Commands.runOnce(() -> intakeArm.setStowedPosition(), intakeArm)
        .withName("Set Stowed Position")
        .ignoringDisable(true);
  }

  public void periodic() {
    field.setRobotPose(drivetrain.getPosition());
  }
}
