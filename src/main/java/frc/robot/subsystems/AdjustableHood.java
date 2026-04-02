/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.ADJUSTABLE_HOOD_ID;
import static frc.robot.Constants.RobotConstants.NOMINAL_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import frc.robot.RobotSelector;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Map;

/** A 2 Angle hood has a extended Angle for passing and a stowed Angle for shooting. */
@DashboardDefinition
public class AdjustableHood extends SubsystemBase implements ActiveSubsystem {

  private static final DataLog LOG = DataLogManager.getLog();

  private static final MotorParameters MOTOR_PARAMS =
      RobotPreferences.ROBOT_TYPE.selectOrDefault(
          Map.of(
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX60,
              RobotSelector.PracticeRobot2026, MotorParameters.NullMotor),
          MotorParameters.NullMotor);

  private static final double TOLERANCE = Units.degreesToRadians(2.0);

  private static final double GEAR_RATIO = 25;
  private static final double RADIANS_PER_ROTATION = (2 * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (RADIANS_PER_ROTATION * (MOTOR_PARAMS.getFreeSpeedRPM()) / 60.0);

  public static final double STOW_ANGLE = Units.degreesToRadians(75); // TODO: Change angles
  public static final double PASSING_ANGLE = Units.degreesToRadians(50);
  public static final double MIN_ANGLE = PASSING_ANGLE; // TODO: Change angles
  public static final double MAX_ANGLE = STOW_ANGLE;

  public static final double KS = MOTOR_PARAMS.getKs();
  public static final double KV = (NOMINAL_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  private final SimpleMotorFeedforward feedforwardController;

  private final TalonFX talonFX = new TalonFX(ADJUSTABLE_HOOD_ID);
  //TODO: Determine direction
  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/AdjustableHood/Motor", talonFX, CLOCKWISE_POSITIVE, BRAKE, RADIANS_PER_ROTATION); 

  private final RelativeEncoder encoder = motor.getEncoder();

  private double currentAngle;
  private double goalAngle = STOW_ANGLE;
  private boolean enabled;

  @DashboardCommand(
      title = "Set Extended Angle",
      column = 2,
      row = 0,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setExtendedAngleCommand =
      Commands.runOnce(this::setPassingAngle, this)
          .withName("Set Extended Angle")
          .ignoringDisable(true);

  @DashboardTextDisplay(
      title = "Test Goal Angle",
      column = 2,
      row = 1,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalAngle = 0;

  @DashboardCommand(
      title = "Set Test Goal Angle",
      column = 2,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalAngleCommand =
      Commands.runOnce(() -> setGoalAngle(Math.toRadians(testGoalAngle)), this)
          .withName("Set Goal Angle");

  @DashboardCommand(
      title = "Disable",
      column = 2,
      row = 3,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommand =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  private final PIDControllerPreference pidController;

  private final DoubleLogEntry logCurrentAngle = new DoubleLogEntry(LOG, "Hood/Current Angle");
  private final DoubleLogEntry logGoalAngle = new DoubleLogEntry(LOG, "Hood/Goal Angle");

  /** Creates a new AdjustableHood. */
  public AdjustableHood() {

    pidController = new PIDControllerPreference("AdjustableHood", "PID Controller", 1, 0, 0);
    feedforwardController = new SimpleMotorFeedforward(KS, KV);

    resetHoodAngle(STOW_ANGLE);
  }

  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    goalAngle = angle;
    enabled = true;
    pidController.reset();
  }

  public void setPassingAngle() {
    setGoalAngle(PASSING_ANGLE);
  }

  public void setStowedAngle() {
    setGoalAngle(STOW_ANGLE);
  }

  private void resetHoodAngle(double angleRadians) {
    encoder.setPosition(angleRadians);
    goalAngle = angleRadians;
    logCurrentAngle.append(currentAngle);
    logGoalAngle.append(goalAngle);
  }

  /**
   * {@return the current intake arm angle in degrees} The angle is relative to horizontal with
   * positive values upward.
   */
  @DashboardRadialGauge(
      title = "Current Angle",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      startAngle = -180,
      endAngle = 180,
      min = -180,
      max = 180,
      numberOfLabels = 0,
      wrapValue = true)
  public double getCurrentAngleDegrees() {
    return Math.toDegrees(currentAngle);
  }

  /** {@return the current angle in radians} */
  public double getCurrentAngle() {
    return currentAngle;
  }

  /**
   * {@return the goal angle in degress} The angle is relative to horizontal with positive values
   * upward.
   */
  @DashboardTextDisplay(
      title = "Goal Angle",
      column = 0,
      row = 2,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_ONLY)
  public double getGoalAngleDegrees() {
    return Math.toDegrees(goalAngle);
  }

  /**
   * {@return whether the intake arm is near the specified angle}
   *
   * @param goalAngle The angle to check, in radians.
   */
  private boolean atAngle(double goalAngle) {
    return Math.abs(goalAngle - currentAngle) <= TOLERANCE;
  }

  /** {@return whether the intake arm is near the goal angle} */
  public boolean atGoalAngle() {
    return atAngle(this.goalAngle);
  }

  /** Polls sensors and logs telemetry. */
  private void updateTelemetry() {
    currentAngle = encoder.getPosition();
    motor.logTelemetry();
  }

  @Override
  public void disable() {
    enabled = false;
    motor.disable();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  @Override
  @DashboardBooleanBox(title = "Enabled", column = 0, row = 3, width = 1, height = 1)
  public boolean isEnabled() {
    return enabled;
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if ((goalAngle == STOW_ANGLE || goalAngle == PASSING_ANGLE)) {
      if (atGoalAngle()) {
        disable();
      } else if (!enabled) {
        setGoalAngle(goalAngle);
      }
    }
    if (enabled) {
      double feedforward = feedforwardController.calculate(goalAngle);
      double feedback = pidController.calculate(currentAngle, goalAngle);
      double voltage = feedforward + feedback;
      motor.setVoltage(voltage);
    }
  }
}
