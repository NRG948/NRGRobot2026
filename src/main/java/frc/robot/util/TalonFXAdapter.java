/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ParentConfiguration;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.HashMap;
import java.util.function.Function;

/** A motor controller implementation based on the CTR Electronics TalonFX controller. */
public final class TalonFXAdapter implements MotorController {
  private static final int NUM_RETRIES = 5;

  private static final DataLog LOG = DataLogManager.getLog();

  @SuppressWarnings("unused")
  private final String logPrefix;

  private final TalonFX talonFX;
  private double distancePerRotation;
  public final StatusSignal<Current> supplyCurrent;
  public final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final DoubleLogEntry logSupplyCurrent;
  private final DoubleLogEntry logStatorCurrent;
  private final DoubleLogEntry logTemperature;

  public static final HashMap<Integer, TalonFXAdapter> motors =
      new HashMap<Integer, TalonFXAdapter>();

  public static final SendableChooser<Integer> motorChooser = new SendableChooser<Integer>();

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

    BaseStatusSignal.setUpdateFrequencyForAll(10.0, this.supplyCurrent, this.statorCurrent);

    String name = String.format("%s/TalonFX-%d", logPrefix, talonFX.getDeviceID());

    this.logSupplyCurrent = new DoubleLogEntry(LOG, name + "/SupplyCurrent");
    this.logStatorCurrent = new DoubleLogEntry(LOG, name + "/StatorCurrent");
    this.logTemperature = new DoubleLogEntry(LOG, name + "/Temperature");

    TalonFXAdapter.motorChooser.addOption(name, talonFX.getDeviceID());
    TalonFXAdapter.motors.put(talonFX.getDeviceID(), this);
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

  /**
   * Sets the MotionMagic voltage
   *
   * @param voltage The voltage and optional feedforward to set for the MotionMagic control mode.
   */
  public void setControl(MotionMagicVoltage voltage) {
    talonFX.setControl(voltage);
  }

  /**
   * Sets the MotionMagic velocity
   *
   * @param velocity The velocity and optional feedforward to set for the MotionMagic control mode.
   */
  public void setControl(MotionMagicVelocityVoltage velocity) {
    talonFX.setControl(velocity);
  }

  /**
   * Gets a configuration from the TalonFX with retries and error handling.
   *
   * @param configClass The class of the configuration to retrieve.
   * @param refresher A function that refreshes the configuration.
   * @param <T> The type of the configuration.
   * @return The retrieved configuration.
   * @throws MotorConfigException If the configuration cannot be retrieved.
   */
  private <T extends ParentConfiguration> T getConfig(
      Class<T> configClass, Function<T, StatusCode> refresher) throws MotorConfigException {
    T config;
    StatusCode status = StatusCode.OK;

    try {
      config = configClass.getDeclaredConstructor().newInstance();
    } catch (Exception e) {
      String errorMessage =
          String.format(
              "Failed to create instance of config class %s: %s",
              configClass.getSimpleName(), e.getMessage());
      DriverStation.reportError(errorMessage, true);
      throw new MotorConfigException(errorMessage, e);
    }

    for (int i = 0; i < NUM_RETRIES; i++) {
      status = refresher.apply(config);
      if (status.isOK()) {
        return config;
      }
    }

    String errorMessage =
        String.format(
            "Failed to get %s from ID %d: %s (%s)",
            configClass.getSimpleName(),
            talonFX.getDeviceID(),
            status.getDescription(),
            status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  /**
   * Applies a configuration to the TalonFX with retries and error handling.
   *
   * @param config The configuration to apply.
   * @param applier A function that applies the configuration.
   * @param <T> The type of the configuration.
   * @throws MotorConfigException If the configuration cannot be applied.
   */
  private <T extends ParentConfiguration> void applyConfig(
      T config, Function<T, StatusCode> applier) throws MotorConfigException {
    StatusCode status = StatusCode.OK;

    for (int i = 0; i < NUM_RETRIES; i++) {
      status = applier.apply(config);
      if (status.isOK()) {
        return;
      }
    }

    String errorMessage =
        String.format(
            "Failed to apply %s to ID %d: %s (%s)",
            config.getClass().getSimpleName(),
            talonFX.getDeviceID(),
            status.getDescription(),
            status.getName());

    DriverStation.reportError(errorMessage, true);

    throw new MotorConfigException(errorMessage);
  }

  /**
   * Gets the current motor output configuration.
   *
   * @return The current motor output configuration.
   * @throws MotorConfigException If the configuration cannot be retrieved.
   */
  public MotorOutputConfigs getMotorOutputConfig() throws MotorConfigException {
    return getConfig(MotorOutputConfigs.class, talonFX.getConfigurator()::refresh);
  }

  /**
   * Applies the motor output configuration.
   *
   * @param motorOutputConfigs The motor output configuration to apply.
   * @throws MotorConfigException If the configuration is invalid or we fail to apply it for any
   *     reason.
   */
  public void applyMotorOutputConfig(MotorOutputConfigs motorOutputConfigs)
      throws MotorConfigException {
    applyConfig(motorOutputConfigs, talonFX.getConfigurator()::apply);
  }

  /**
   * Gets the current Talon FX configuration.
   *
   * @return The current Talon FX configuration.
   * @throws MotorConfigException If the configuration cannot be retrieved.
   */
  public TalonFXConfiguration getTalonFXConfiguration() throws MotorConfigException {
    return getConfig(TalonFXConfiguration.class, talonFX.getConfigurator()::refresh);
  }

  /**
   * Applies a full TalonFX configuration.
   *
   * @param config the TalonFX configuration to apply.
   * @throws MotorConfigException If the configuration is invalid or we fail to apply it for any
   *     reason.
   */
  public void applyTalonFXConfiguration(TalonFXConfiguration config) throws MotorConfigException {
    applyConfig(config, talonFX.getConfigurator()::apply);
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
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = config.enableCurrentLimit();
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = config.enableCurrentLimit();
    talonFXConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit();
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit();
    applyTalonFXConfiguration(talonFXConfig);
    return this;
  }

  /**
   * Gets the supply current of the currently selected motor in the motor chooser.
   *
   * <p>This is useful for debugging and monitoring the current draw of the motors during operation.
   * Note that this method will return 0 if no motor is selected or if the selected motor cannot be
   * found in the motors map.
   *
   * @return The supply current of the currently selected motor, or 0 if no motor is selected or
   *     found.
   */
  public static double getSelectedMotorSupplyCurrent() {
    TalonFXAdapter motor = motors.get(motorChooser.getSelected());
    return motor == null ? 0.0 : motor.supplyCurrent.getValueAsDouble();
  }

  /**
   * Gets the stator current of the currently selected motor in the motor chooser.
   *
   * <p>This is useful for debugging and monitoring the current draw of the motors during operation.
   * Note that this method will return 0 if no motor is selected or if the selected motor cannot be
   * found in the motors map.
   *
   * @return The stator current of the currently selected motor, or 0 if no motor is selected or
   *     found.
   */
  public static double getSelectedMotorStatorCurrent() {
    TalonFXAdapter motor = motors.get(motorChooser.getSelected());
    return motor == null ? 0.0 : motor.statorCurrent.getValueAsDouble();
  }
}
