/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;

import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.RobotPreferences;
import frc.robot.RobotSelector;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import java.util.Map;

@DashboardDefinition
public final class Indexer extends SubsystemBase implements ActiveSubsystem {
  private static final MotorParameters MOTOR =
      RobotPreferences.ROBOT_TYPE.selectOrDefault(
          Map.of(
              RobotSelector.AlphaRobot2026, MotorParameters.NullMotor,
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX60,
              RobotSelector.PracticeRobot2026, MotorParameters.KrakenX60),
          MotorParameters.NullMotor);

  private static final double BAR_DIAMETER = Units.inchesToMeters(1.25);
  private static final double GEAR_RATIO = 1.0;
  private static final double METERS_PER_REVOLUTION = (BAR_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY = MOTOR.getFreeSpeedRPM() * METERS_PER_REVOLUTION / 60;
  private static final double FEED_VELOCITY = 1.8;
  private static final double OUTFEED_VELOCITY = -2.0;

  private final MotorController shooterIndexerMotor =
      MOTOR.newController(
          "/Indexer/Shooter Motor",
          CANID.SHOOTER_INDEXER_ID,
          MotorDirection.CLOCKWISE_POSITIVE,
          MotorIdleMode.BRAKE,
          0);
  private final RelativeEncoder shooterIndexerEncoder = shooterIndexerMotor.getEncoder();

  private final MotorController hopperIndexerMotor =
      shooterIndexerMotor.createFollower("/Indexer/Hopper Motor", CANID.HOPPER_INDEXER_ID, false);

  private final double KS = MOTOR.getKs();
  private final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);

  @DashboardTextDisplay(title = "Goal Velocity (m/s)", column = 0, row = 2, width = 2, height = 1)
  private double goalVelocity = 0;

  @DashboardRadialGauge(
      title = "Current Velocity (m/s)",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      min = -15.9593,
      max = 15.9593)
  private double currentVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Goal Velocity (m/s)",
      column = 2,
      row = 0,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalVelocity = 0;

  @DashboardCommand(
      title = "Set Test Goal Velocity",
      column = 2,
      row = 1,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalVelocityCommand =
      Commands.runOnce(() -> setGoalVelocity(testGoalVelocity), this)
          .withName("Set Test Goal Velocity");

  @DashboardCommand(
      title = "Disable",
      column = 2,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommands =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  @DashboardPIDController(title = "PID Controller", column = 4, row = 0, width = 2, height = 3)
  private final PIDControllerPreference pidController =
      new PIDControllerPreference("Indexer", "PID Controller", 1, 0, 0);

  /** Creates a new Indexer. */
  public Indexer() {}

  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
  }

  public void feed() {
    setGoalVelocity(FEED_VELOCITY);
  }

  public void outFeed() {
    setGoalVelocity(OUTFEED_VELOCITY);
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    shooterIndexerMotor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    shooterIndexerMotor.setIdleMode(idleMode);
  }

  @Override
  public boolean isEnabled() {
    return goalVelocity != 0;
  }

  @Override
  public void periodic() {
    updateTelemetry();

    if (goalVelocity != 0) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double voltage = feedforward + feedback;
      shooterIndexerMotor.setVoltage(voltage);

    } else {
      shooterIndexerMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    currentVelocity = shooterIndexerEncoder.getVelocity();
    shooterIndexerMotor.logTelemetry();
    hopperIndexerMotor.logTelemetry();
  }

  @DashboardTextDisplay(
      title = "Current Velocity (m/s)",
      column = 0,
      row = 3,
      width = 2,
      height = 1)
  public double getCurrentVelocity() {
    return currentVelocity;
  }
}
