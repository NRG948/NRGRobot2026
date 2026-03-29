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
import com.nrg948.dashboard.annotations.DashboardSingleColorView;
import com.nrg948.dashboard.annotations.DashboardSplitButtonChooser;
import com.nrg948.dashboard.model.GameField;
import com.nrg948.util.Colors;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.util.HubState;
import frc.robot.util.MatchUtil;
import java.util.Optional;

@DashboardDefinition
public final class RobotOperator {
  private static final String BLACK_HEX_STRING = Colors.BLACK.toHexString();
  private static final double BLINK_DURATION = 1.0 / 3.0;

  private final Swerve drivetrain;
  private final IntakeArm intakeArm;
  private final Optional<AprilTag> frontLeftCamera;
  private final Optional<AprilTag> frontRightCamera;

  private HubState hubState = HubState.INACTIVE;
  private Timer blinkTimer = new Timer();
  private boolean blinkOn = true;

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

  @DashboardAlerts(title = "Alerts", column = 0, row = 3, width = 5, height = 2)
  private final Alert[] alerts = new Alert[] {Autos.getInvalidAutoAlert()};

  public RobotOperator(Subsystems subsystems) {
    drivetrain = subsystems.drivetrain;
    intakeArm = subsystems.intakeArm;
    frontLeftCamera = subsystems.frontLeftCamera;
    frontRightCamera = subsystems.frontRightCamera;
  }

  @DashboardField(
      title = "Field",
      row = 0,
      column = 0,
      height = 3,
      width = 7,
      game = GameField.REBUILT)
  private Field2d field = new Field2d();

  @DashboardMatchTime(title = "Shift Time", row = 0, column = 9, width = 3, height = 2)
  public static double getShiftTime() {
    return MatchUtil.getShiftTimeRemaining();
  }

  @DashboardBooleanBox(
      title = "Front Left Camera Connected",
      column = 5,
      row = 3,
      width = 1,
      height = 1)
  public boolean frontLeftCameraIsConnected() {
    return frontLeftCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(
      title = "Front Right Camera Connected",
      column = 6,
      row = 3,
      width = 1,
      height = 1)
  public boolean frontRightCameraIsConnected() {
    return frontRightCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardSingleColorView(title = "On Shift", column = 7, row = 0, width = 2, height = 2)
  public String onShiftIndicatorColor() {
    if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed(BLINK_DURATION)) {
      blinkOn = !blinkOn;
    }

    return blinkOn ? hubState.getColor() : BLACK_HEX_STRING;
  }

  /** Called in periodic() to update to the hub state. */
  private void updateHubState() {
    if (MatchUtil.isAutonomous()) {
      setHubState(HubState.ACTIVE);
      return;
    }

    if (!MatchUtil.isTeleop()) {
      setHubState(HubState.INACTIVE);
      return;
    }

    double matchTime = MatchUtil.getMatchTimeRemaining();

    if (matchTime < 0) {
      setHubState(HubState.INACTIVE);
    }

    switch (hubState) {
      case ACTIVE:
        if (!MatchUtil.isHubActiveAt(
            matchTime - HubState.PREPARING_TO_DISABLE_10_SEC.getDeltaTime())) {
          setHubState(HubState.PREPARING_TO_DISABLE_10_SEC);
        } else if (matchTime <= HubState.NEARING_END_OF_MATCH.getDeltaTime()) {
          setHubState(HubState.NEARING_END_OF_MATCH);
        }
        break;
      case INACTIVE:
        if (MatchUtil.isHubActiveAt(
            matchTime - HubState.PREPARING_SHOOTING_DISABLED.getDeltaTime())) {
          setHubState(HubState.PREPARING_SHOOTING_DISABLED);
        }
        break;
      case PREPARING_SHOOTING_DISABLED:
        if (MatchUtil.isHubActiveAt(
            matchTime - HubState.PREPARING_SHOOTING_ENABLED.getDeltaTime())) {
          setHubState(HubState.PREPARING_SHOOTING_ENABLED);
        }
        break;
      case PREPARING_SHOOTING_ENABLED:
        if (MatchUtil.isHubActiveAt(matchTime)) {
          setHubState(HubState.ACTIVE);
        }
        break;
      case PREPARING_TO_DISABLE_5_SEC:
        if (!MatchUtil.isHubActiveAt(matchTime)) {
          setHubState(HubState.INACTIVE);
        }
        break;
      case PREPARING_TO_DISABLE_10_SEC:
        if (!MatchUtil.isHubActiveAt(
            matchTime - HubState.PREPARING_TO_DISABLE_5_SEC.getDeltaTime())) {
          setHubState(HubState.PREPARING_TO_DISABLE_5_SEC);
        }
        break;
      case NEARING_END_OF_MATCH:
        if (matchTime <= 0) {
          setHubState(HubState.INACTIVE);
        }
        break;
    }
  }

  /**
   * Sets the hub state.
   *
   * @param newHubState The new state.
   */
  private void setHubState(HubState newHubState) {
    if (hubState == newHubState) {
      return;
    }

    hubState = newHubState;
    if (hubState.blink()) {
      if (!blinkTimer.isRunning()) {
        blinkTimer.reset();
        blinkTimer.start();
        blinkOn = false;
      }
    } else {
      blinkTimer.stop();
      blinkOn = true;
    }
  }

  public HubState getHubState() {
    return hubState;
  }

  @DashboardBooleanBox(title = "Within Range", column = 7, row = 2, width = 2, height = 1)
  public boolean isWithinShootingRange() {
    return drivetrain.getDistanceToTarget() <= Shooter.MAX_SHOOTING_DISTANCE;
  }

  @DashboardBooleanBox(title = "Aligned to Hub", column = 7, row = 3, width = 2, height = 1)
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
      column = 5,
      row = 4,
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
    updateHubState();
  }

  public void teleopInit() {
    setHubState(MatchUtil.isHubActive() ? HubState.ACTIVE : HubState.INACTIVE);
  }

  public void autonomousInit() {
    setHubState(HubState.ACTIVE);
  }
}
