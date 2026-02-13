/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;

import java.util.Map;

import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;

import com.ctre.phoenix6.hardware.TalonFX;

import com.nrg948.preferences.PIDControllerPreference;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import frc.robot.RobotContainer.RobotSelector;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.TalonFXAdapter;
import frc.robot.util.RelativeEncoder;
import static frc.robot.util.MotorIdleMode.BRAKE;


public class Elevator extends SubsystemBase implements ActiveSubsystem {
   // TODO: put actual values
    private static final double GEAR_RATIO = 1; 
  private static final double SPROCKET_DIAMETER = 0.05; // meters
  private static final double MASS = 100; // kilograms
  private static final double POSITION_ERROR_MARGIN = 0.05; 
  private static final double METERS_PER_REVOLUTION = (SPROCKET_DIAMETER * Math.PI) / GEAR_RATIO;

 private static final MotorParameters MOTOR =
      RobotPreferences.ROBOT_TYPE.selectOrDefault(
          Map.of(
              RobotSelector.AlphaRobot2026, MotorParameters.NullMotor,
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX60,
              RobotSelector.PracticeRobot2026, MotorParameters.KrakenX60),
          MotorParameters.NullMotor);
  
  //TODO: Change CAN ID
  private final MotorController motor =
      MOTOR.newController(
          "/Intake/Motor", 1, COUNTER_CLOCKWISE_POSITIVE, BRAKE, METERS_PER_REVOLUTION);
  
  private final RelativeEncoder encoder = motor.getEncoder();
 

  //TODO: Change PID and dashboard value     
  @DashboardPIDController(title = "PID", column = 7, row = 0, width = 2, height = 3)
  private final PIDControllerPreference pidController =
      new PIDControllerPreference("Elevator", "PID Controller", 1, 0, 0);

  // Trapezoid profile values
  private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private final Mechanism2d mechanism2d = new Mechanism2d(0.5, 1.0);
  private final MechanismRoot2d mechanismRoot2d = mechanism2d.getRoot("Elevator Root", 0, 0);

  private ElevatorSim simElevator =
      new ElevatorSim(MOTOR_PARAMS, GEAR_RATIO, MASS, SPROCKET_DIAMETER / 2, 0, 1, true, 0);
  private boolean atUpperLimit;
  private boolean atLowerLimit;

  //TODO: Change values
  private static final double MAX_HEIGHT = 20; // meters
  private static final double MIN_HEIGHT = 1; // meters
  private static final double DISABLE_HEIGHT = MIN_HEIGHT + 0.01;
  private static final double MAX_SPEED =
      (MOTOR_PARAMS.freeSpeedRadPerSec / (2 * Math.PI)) * METERS_PER_REVOLUTION; // m/s
  private static final double MAX_ACCELERATION =
      (MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO) / (SPROCKET_DIAMETER * MASS); // m/s^2
  private static final ExponentialProfile.Constraints EXPONENTIAL_CONSTRAINTS =
      ExponentialProfile.Constraints.fromCharacteristics(
          MAX_BATTERY_VOLTAGE, MAX_SPEED / 2, MAX_ACCELERATION / 64);

  private final MechanismLigament2d elevatorMech2d =
      mechanismRoot2d.append(new MechanismLigament2d("Elevator", 0, 90));
  private static final double POSITION_ERROR_TIME = 2.0; // seconds

  private final Timer stuckTimer = new Timer();
  
  // TODO: Change Feedforward constants
  private static final double KS = 0;
  private static final double KV = 0;
  private static final double KA = 0;
  private static final double KG = 9.81 * KA;
  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);


  private boolean isSeekingGoal;
  private boolean hasError;

  private final ExponentialProfile.State currentState = new ExponentialProfile.State();
  private final ExponentialProfile.State goalState = new ExponentialProfile.State();
  private ExponentialProfile.State lastState = currentState;

  private ElevatorLevel currentElevatorLevel = ElevatorLevel.STOWED;

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setIdleMode(MotorIdleMode idleMode) {
    // Do not ever put the elevator in COAST mode or else it will crash down.
  }

  public void setGoalHeight(ElevatorLevel level) {
    setGoalHeight(level.getElevatorHeight());
  } 

  /**
   * Sets elevator goal height in meters.
   *
   * @param height the desired height of the elevator in meters.
   */
  private void setGoalHeight(double height) {
    isSeekingGoal = true;
    goalState.position = height;
    goalState.velocity = 0;
    
    lastState = currentState;

    /*pidController.setPID(KP.getValue(), KI.getValue(), KD.getValue());
    pidController.reset();*/

  }


  

  public void disable() {}

  private void updateTelemetry() {
    if (RobotBase.isReal()) {
      currentState.position = encoder.getPosition();
      currentState.velocity = encoder.getVelocity();
    } else {
      currentState.position = simElevator.getPositionMeters();
      currentState.velocity = simElevator.getVelocityMetersPerSecond();
    }
    currentState.position += MIN_HEIGHT;

    atUpperLimit = currentState.position >= MAX_HEIGHT;
    atLowerLimit = currentState.position <= DISABLE_HEIGHT;

    checkError();

    elevatorMech2d.setLength(currentState.position);
}

private void checkError() {
    if (MathUtil.isNear(goalState.position, currentState.position, POSITION_ERROR_MARGIN)) {
      stuckTimer.stop();
      stuckTimer.reset();
    } else {
      if (!stuckTimer.isRunning()) {
        stuckTimer.restart();
      }
    }
    hasError = stuckTimer.hasElapsed(POSITION_ERROR_TIME);
  }


}
