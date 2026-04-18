/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/** A motor controller implementation based on the CTR Electronics TalonFX controller. */
public final class TalonFXAdapter implements MotorController {
  private static final DataLog LOG = DataLogManager.getLog();

  @SuppressWarnings("unused")
  private final String logPrefix;

  private final TalonFX talonFX;
  private double distancePerRotation;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final DoubleLogEntry logSupplyCurrent;
  private final DoubleLogEntry logStatorCurrent;
  private final DoubleLogEntry logTemperature;

  /**
   * Constructs a TalonFXAdapter.
   *
   * @param logPrefix The prefix for the log entries.
   * @param talonFX The TalonFX object to adapt.
   */
  public TalonFXAdapter(String logPrefix, TalonFX talonFX) {
    this.logPrefix = logPrefix;
    this.talonFX = talonFX;
    this.supplyCurrent = talonFX.getSupplyCurrent();
    this.statorCurrent = talonFX.getStatorCurrent();
    this.temperature = talonFX.getDeviceTemp();

    String name = String.format("%s/TalonFX-%d", logPrefix, talonFX.getDeviceID());

    this.logSupplyCurrent = new DoubleLogEntry(LOG, name + "/SupplyCurrent");
    this.logStatorCurrent = new DoubleLogEntry(LOG, name + "/StatorCurrent");
    this.logTemperature = new DoubleLogEntry(LOG, name + "/Temperature");
  }

  @Override
  public void set(double speed) {
    talonFX.set(speed);
  }

  @Override
  public double get() {
    return talonFX.get();
  }

  @Override
  public void setVoltage(Voltage outputVoltage) {
    talonFX.setVoltage(outputVoltage.magnitude());
  }

  @Override
  public void setInverted(boolean isInverted) {
    try {
      var motorOutputConfigs = getMotorOutputConfig();
      motorOutputConfigs.Inverted =
          isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      applyMotorOutputConfig(motorOutputConfigs);
    } catch (MotorConfigException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public boolean getInverted() {
    MotorOutputConfigs motorOutputConfigs;
    try {
      motorOutputConfigs = getMotorOutputConfig();
      return motorOutputConfigs.Inverted == InvertedValue.Clockwise_Positive;
    } catch (MotorConfigException e) {

    }

    return false;
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    try {
      var motorOutputConfigs = getMotorOutputConfig();
      motorOutputConfigs.NeutralMode = idleMode.forTalonFX();
      applyMotorOutputConfig(motorOutputConfigs);
    } catch (MotorConfigException e) {

    }
  }

  @Override
  public void disable() {
    talonFX.disable();
  }

  @Override
  public void stopMotor() {
    talonFX.stopMotor();
  }

  @Override
  public MotorController createFollower(
      String logPrefix, int deviceID, boolean isInvertedFromLeader) throws MotorConfigException {
    TalonFX follower = new TalonFX(deviceID, talonFX.getNetwork());
    TalonFXAdapter followerAdapter = new TalonFXAdapter(logPrefix, follower);

    // Get the motor output configuration from the leader and apply it to the follower.
    followerAdapter.applyMotorOutputConfig(getMotorOutputConfig());

    // Configure the follower to follow the leader.
    Follower followerConfig =
        new Follower(
            talonFX.getDeviceID(),
            isInvertedFromLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);

    follower.setControl(followerConfig);

    return followerAdapter;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return new TalonFXEncoderAdapter(talonFX, distancePerRotation);
  }

  @Override
  public LimitSwitch getForwardLimitSwitch() {
    return new TalonFXLimitSwitchAdapter<ForwardLimitValue>(
        talonFX.getForwardLimit(), ForwardLimitValue.ClosedToGround);
  }

  @Override
  public LimitSwitch getReverseLimitSwitch() {
    return new TalonFXLimitSwitchAdapter<ReverseLimitValue>(
        talonFX.getReverseLimit(), ReverseLimitValue.ClosedToGround);
  }

  @Override
  public void logTelemetry() {
    logSupplyCurrent.append(this.supplyCurrent.refresh().getValueAsDouble());
    logStatorCurrent.append(this.statorCurrent.refresh().getValueAsDouble());
    logTemperature.append(this.temperature.refresh().getValueAsDouble());
  }

  public MotorOutputConfigs getMotorOutputConfig() throws MotorConfigException {
    var motorOutputConfig = new MotorOutputConfigs();
    StatusCode status = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      status = talonFX.getConfigurator().refresh(motorOutputConfig);
      if (status.isOK()) {
        return motorOutputConfig;
      }
    }

    String errorMessage =
        String.format(
            "ERROR: Failed to get motor output config from ID %d: %s (%s)",
            talonFX.getDeviceID(), status.getDescription(), status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  public void applyMotorOutputConfig(MotorOutputConfigs motorOutputConfigs)
      throws MotorConfigException {
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < 5; i++) {
      status = talonFX.getConfigurator().apply(motorOutputConfigs);
      if (status.isOK()) {
        return;
      }
    }
    String errorMessage =
        String.format(
            "ERROR: Failed to apply motor output config to ID %d: %s (%s)",
            talonFX.getDeviceID(), status.getDescription(), status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  public TalonFXConfiguration getTalonFXConfiguration() throws MotorConfigException {
    var talonFXConfig = new TalonFXConfiguration();
    StatusCode status = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      status = talonFX.getConfigurator().refresh(talonFXConfig);
      if (status.isOK()) {
        return talonFXConfig;
      }
    }

    String errorMessage =
        String.format(
            "ERROR: Failed to get TalonFX config from ID %d: %s (%s)",
            talonFX.getDeviceID(), status.getDescription(), status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  /**
   * Applies a full TalonFX configuration with up to 5 retries.
   *
   * @param config the TalonFX configuration to apply.
   * @return true if the configuration was successfully applied, false if all retries were
   *     exhausted.
   * @throws MotorConfigException when it fails to configure the motor.
   */
  public void applyTalonFXConfiguration(TalonFXConfiguration config) throws MotorConfigException {
    StatusCode status = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      status = talonFX.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }

    String errorMessage =
        String.format(
            "ERROR: Failed to apply TalonFX config to ID %d: %s (%s)",
            talonFX.getDeviceID(), status.getDescription(), status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  /**
   * Sets the MotionMagic voltage
   *
   * @param voltage
   */
  public void setControl(MotionMagicVoltage voltage) {
    talonFX.setControl(voltage);
  }

  public void setControl(MotionMagicVelocityVoltage request) {
    talonFX.setControl(request);
  }

  @Override
  public MotorController apply(MotorConfig config) throws MotorConfigException {
    var talonFXConfig = getTalonFXConfiguration();
    talonFXConfig.MotorOutput.Inverted = config.direction().forTalonFX();
    talonFXConfig.MotorOutput.NeutralMode = config.idleMode().forTalonFX();
    talonFXConfig.Feedback.SensorToMechanismRatio = 1.0;
    applyTalonFXConfiguration(talonFXConfig);
    distancePerRotation = config.distancePerRotation();
    return this;
  }

  @Override
  public MotorController apply(MotorCurrentConfig config) throws MotorConfigException {
    var talonFXConfig = getTalonFXConfiguration();
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit();
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit();
    applyTalonFXConfiguration(talonFXConfig);
    return this;
  }
}
