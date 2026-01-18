/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

public class Shooter extends SubsystemBase implements ActiveSubsystem {

  private static final DataLog LOG = DataLogManager.getLog();

  private final MotorDirection motorDirection = CLOCKWISE_POSITIVE;
  private final TalonFXAdapter lowerMotor =
      new TalonFXAdapter("/lowerMotor", new TalonFX(0, "rio"), motorDirection, BRAKE, 0);
  private final MotorController lowerMotor2 = lowerMotor.createFollower(0, true);
  private final TalonFXAdapter upperMotor =
      new TalonFXAdapter("/upperMotor", new TalonFX(0, "rio"), motorDirection, BRAKE, 0);
  private final MotorController upperMotor2 = upperMotor.createFollower(0, true);

  private static final double GEAR_RATIO = 9;
  private static final double MAX_RPM = KrakenX60.getFreeSpeedRPM() / GEAR_RATIO;

  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_RPM;

  private final RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  private final RelativeEncoder upperEncoder = upperMotor.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController lowerPIDController = new PIDController(1, 0, 0);
  private final PIDController upperPIDController = new PIDController(1, 0, 0);

  private double lowerGoalRPM = 0;
  private double upperGoalRPM = 0;
  private double lowerCurrentRPM = 0; // Different RPM for upper and lower pairs of motors
  private double upperCurrentRPM = 0;

  private DoubleLogEntry logLowerGoalRPM = new DoubleLogEntry(LOG, "/Shooter/lowerGoalRPM");
  private DoubleLogEntry logUpperGoalRPM = new DoubleLogEntry(LOG, "/Shooter/upperGoalRPM");
  private DoubleLogEntry logLowerCurrentRPM = new DoubleLogEntry(LOG, "/Shooter/lowerCurrentRPM");
  private DoubleLogEntry logUpperCurrentRPM = new DoubleLogEntry(LOG, "/Shooter/upperCurrentRPM");

  /** Creates a new Intake. */
  public Shooter() {}

  public void setGoalRPM(double lowerGoalRPM, double upperGoalRPM) {
    this.lowerGoalRPM = lowerGoalRPM;
    this.upperGoalRPM = upperGoalRPM;
    logLowerGoalRPM.append(lowerGoalRPM);
    logUpperGoalRPM.append(upperGoalRPM);
  }

  @Override
  public void disable() {
    lowerGoalRPM = 0;
    upperGoalRPM = 0;
    logLowerGoalRPM.append(0);
    logUpperGoalRPM.append(0);
    lowerMotor.stopMotor();
    lowerMotor2.stopMotor();
    upperMotor.stopMotor();
    upperMotor2.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    lowerMotor.setIdleMode(idleMode);
    lowerMotor2.setIdleMode(idleMode);
    upperMotor.setIdleMode(idleMode);
    upperMotor2.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if (lowerGoalRPM != 0) {
      double feedforward = this.feedforward.calculate(lowerGoalRPM);
      double feedback = lowerPIDController.calculate(lowerCurrentRPM, lowerGoalRPM);
      double motorVoltage = feedforward + feedback;
      lowerMotor.setVoltage(motorVoltage);

    } else {
      lowerMotor.setVoltage(0);
    }
    if (upperGoalRPM != 0) {
      double feedforward = this.feedforward.calculate(upperGoalRPM);
      double feedback = upperPIDController.calculate(upperCurrentRPM, upperGoalRPM);
      double motorVoltage = feedforward + feedback;
      upperMotor.setVoltage(motorVoltage);

    } else {
      upperMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    lowerCurrentRPM = lowerEncoder.getVelocity();
    upperCurrentRPM = upperEncoder.getVelocity();
    logLowerCurrentRPM.append(lowerCurrentRPM);
    logUpperCurrentRPM.append(upperCurrentRPM);
    lowerMotor.logTelemetry();
    lowerMotor2.logTelemetry();
    upperMotor.logTelemetry();
    upperMotor2.logTelemetry();
  }
}
