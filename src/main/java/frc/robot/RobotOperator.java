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
import frc.robot.util.MatchUtil;
import java.util.Optional;

@DashboardDefinition
public final class RobotOperator {
  private static final String BLACK_HEX_STRING = Colors.BLACK.toHexString();
  private static final double BLINK_DURATION = 1.0 / 3.0;

  private enum ShootingReadiness {
    /** When the hub is not active, we are not ready to shoot. */
    NOT_READY(Colors.RED, false, 0),
    /** When the hub is nearing active, we are not ready to shoot. */
    PREPARING_SHOOTING_DISABLED(Colors.RED, true, 5),
    /** When the hub is nearing active, we are ready to shoot. */
    PREPARING_SHOOTING_ENABLED(Colors.YELLOW, true, 2),
    /** When the hub is active, we are ready to shoot. */
    READY(Colors.GREEN, false, 0),
    /** When the hub is nearing not active, we are ready to shoot. */
    PREPARING_TO_DISABLE(Colors.YELLOW, true, 5);

    private final String color;
    private final boolean blink;
    private final double deltaTime;

    private ShootingReadiness(Colors color, boolean blink, double deltaTime) {
      this.color = color.toHexString();
      this.blink = blink;
      this.deltaTime = deltaTime;
    }

    public String getColor() {
      return color;
    }

    public boolean blink() {
      return blink;
    }

    public double getDeltaTime() {
      return deltaTime;
    }
  }

  private final Swerve drivetrain;
  private final IntakeArm intakeArm;
  private final Optional<AprilTag> frontLeftCamera;
  private final Optional<AprilTag> frontRightCamera;

  private ShootingReadiness shootingReadiness = ShootingReadiness.NOT_READY;
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

  @DashboardAlerts(title = "Alerts", column = 0, row = 4, width = 5, height = 2)
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
      height = 4,
      width = 7,
      game = GameField.REBUILT)
  private Field2d field = new Field2d();

  @DashboardMatchTime(title = "Match Time", row = 0, column = 9, width = 3, height = 2)
  public static double getMatchTime() {
    return MatchUtil.getMatchTime();
  }

  @DashboardBooleanBox(
      title = "Front Left Camera Connected",
      column = 5,
      row = 4,
      width = 1,
      height = 1)
  public boolean frontLeftCameraIsConnected() {
    return frontLeftCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardBooleanBox(
      title = "Front Right Camera Connected",
      column = 6,
      row = 4,
      width = 1,
      height = 1)
  public boolean frontRightCameraIsConnected() {
    return frontRightCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  @DashboardSingleColorView(title = "On Shift", column = 7, row = 0, width = 2, height = 2)
  public String onShiftIndicator() {
    if (MatchUtil.isAutonomous()) {
      return ShootingReadiness.READY.getColor();
    }

    double matchTime = getMatchTime();

    switch (shootingReadiness) {
      case READY:
        if (!MatchUtil.isHubActiveAt(
            matchTime - ShootingReadiness.PREPARING_TO_DISABLE.getDeltaTime())) {
          shootingReadiness = ShootingReadiness.PREPARING_TO_DISABLE;
          blinkTimer.reset();
          blinkTimer.start();
          blinkOn = false;
        }
        break;
      case NOT_READY:
        if (MatchUtil.isHubActiveAt(
            matchTime - ShootingReadiness.PREPARING_SHOOTING_DISABLED.getDeltaTime())) {
          shootingReadiness = ShootingReadiness.PREPARING_SHOOTING_DISABLED;
          blinkTimer.reset();
          blinkTimer.start();
          blinkOn = false;
        }
        break;
      case PREPARING_SHOOTING_DISABLED:
        if (MatchUtil.isHubActiveAt(
            matchTime - ShootingReadiness.PREPARING_SHOOTING_ENABLED.getDeltaTime())) {
          shootingReadiness = ShootingReadiness.PREPARING_SHOOTING_ENABLED;
        }
        break;
      case PREPARING_SHOOTING_ENABLED:
        if (MatchUtil.isHubActiveAt(matchTime)) {
          shootingReadiness = ShootingReadiness.READY;
          blinkTimer.stop();
          blinkOn = true;
        }
        break;
      case PREPARING_TO_DISABLE:
        if (!MatchUtil.isHubActiveAt(matchTime)) {
          shootingReadiness = ShootingReadiness.NOT_READY;
          blinkTimer.stop();
          blinkOn = true;
        }
        break;
      default:
        break;
    }

    if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed(BLINK_DURATION)) {
      blinkOn = !blinkOn;
    }

    return blinkOn ? shootingReadiness.getColor() : BLACK_HEX_STRING;
  }

  @DashboardBooleanBox(title = "Within Range", column = 7, row = 2, width = 2, height = 1)
  public boolean isWithinShootingRange() {
    return drivetrain.getDistanceToHub() <= Shooter.MAX_SHOOTING_DISTANCE;
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

  public void teleopInit() {
    shootingReadiness = ShootingReadiness.READY;
    blinkTimer.stop();
    blinkOn = true;
  }

  public void autonomousInit() {
    shootingReadiness = ShootingReadiness.READY;
    blinkTimer.stop();
    blinkOn = true;
  }
}
