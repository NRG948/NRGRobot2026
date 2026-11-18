/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import static com.nrg948.actuator.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static com.nrg948.actuator.MotorIdleMode.BRAKE;
import static com.nrg948.actuator.Motors.KrakenX44;
import static com.nrg948.actuator.Motors.KrakenX60;
import static com.nrg948.drivetrain.SwerveModuleType.MK4I_L2;
import static edu.wpi.first.math.util.Units.lbsToKilograms;
import static frc.robot.Constants.RobotConstants.WHEEL_DIAMETER;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.nrg948.actuator.MotorConfig;
import com.nrg948.actuator.MotorController;
import com.nrg948.actuator.MotorCurrentConfig;
import com.nrg948.actuator.MotorDirection;
import com.nrg948.actuator.Motors;
import com.nrg948.drivetrain.SwerveModuleType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.Gyro;
import frc.robot.util.Pigeon2Gyro;

/** An enum representing the properties for the swerve drive base of a specific robot instance. */
public enum RobotParameters {
  /**
   * Parameters for the 2026 Practice Base robot.
   *
   * <p>
   */
  PracticeBase2026(
      lbsToKilograms(60.7), // TODO Update current PB weight
      0.603,
      0.501,
      MK4I_L2,
      KrakenX60,
      KrakenX44,
      new int[] {1, 2, 3, 4, 5, 6, 7, 8}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {157.76, 148.3, -1.67, -100.72}, // CANCoder offsets
      21),

  /**
   * Parameters for the 2026 Competition Base robot.
   *
   * <p>
   */
  CompetitionBase2026(
      lbsToKilograms(142),
      0.603,
      0.501,
      MK4I_L2,
      KrakenX60,
      KrakenX44,
      new int[] {1, 2, 3, 4, 5, 6, 7, 8}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {-94.48, 122.34, 170, -102.78}, // CANCoder offsets
      21),
  /** Parameters for the 2026 Alpha Base robot. */
  AlphaBase2026(
      lbsToKilograms(60.7),
      0.578,
      0.528,
      MK4I_L2,
      KrakenX60,
      KrakenX44,
      new int[] {1, 2, 3, 4, 5, 6, 7, 8}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {24.00, -60.12, -36.39, -109.69},
      21);

  private static final MotorCurrentConfig STEERING_CURRENT_CONFIG =
      new MotorCurrentConfig(10, 20, true);
  private static final MotorCurrentConfig DRIVE_CURRENT_CONFIG =
      new MotorCurrentConfig(80, 160, true);

  public static class Constants {
    /**
     * A scaling factor used to adjust from theoretical maximums given that any physical system
     * generally cannot achieve them.
     */
    public static final double SCALE_FACTOR = 0.8;
  }

  private final double robotMass;
  private final double wheelDistanceX;
  private final double wheelDistanceY;
  private final SwerveModuleType moduleType;
  private final Motors driveMotor;
  private final Motors steeringMotor;
  private final int[] motorIds;
  private final int[] angleEncoderIds;
  private final double[] angleOffset;
  private final int pigeonID;

  private final LinearVelocity maxDriveSpeed;
  private final LinearAcceleration maxDriveAcceleration;

  private final FeedforwardConstants driveFeedforward;

  public SimpleMotorFeedforward getDriveFeedforward() {
    return new SimpleMotorFeedforward(
        this.driveFeedforward.kS, this.driveFeedforward.kV, this.driveFeedforward.kA);
  }

  public SimpleMotorFeedforward getSteeringFeedforward() {
    return new SimpleMotorFeedforward(
        this.steeringFeedforward.kS, this.steeringFeedforward.kV, this.steeringFeedforward.kA);
  }

  private final AngularVelocity maxSteeringSpeed;
  private final AngularAcceleration maxSteeringAcceleration;

  private final FeedforwardConstants steeringFeedforward;

  private final AngularVelocity maxRotationalSpeed;
  private final AngularAcceleration maxRotationalAcceleration;

  private final Translation2d[] wheelPositions;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveKinematicsConstraint kinematicsConstraint;
  private final TrapezoidProfile.Constraints steeringConstraints;
  private final TrapezoidProfile.Constraints robotRotationalConstraints;

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg including bumpers and battery.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters.
   * @param moduleType The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param driveFeedforward The drive feedforward constants.
   * @param steeringFeedforward The steering feedforward constants.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private RobotParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleType moduleType,
      Motors driveMotor,
      Motors steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      FeedforwardConstants driveFeedForward,
      FeedforwardConstants steeringFeedForward,
      int pigeonID) {
    this.robotMass = robotMass;
    this.wheelDistanceX = wheelDistanceX;
    this.wheelDistanceY = wheelDistanceY;
    this.moduleType = moduleType;
    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.motorIds = motorIds;
    this.angleEncoderIds = angleEncoderIds;
    this.angleOffset = angleOffset;
    this.driveFeedforward = driveFeedForward;
    this.steeringFeedforward = steeringFeedForward;
    this.pigeonID = pigeonID;

    double scaleFactor = Constants.SCALE_FACTOR;

    this.maxDriveSpeed =
        this.moduleType.calculateMaxDriveSpeed(this.driveMotor, WHEEL_DIAMETER).times(scaleFactor);
    this.maxDriveAcceleration =
        this.moduleType
            .calculateMaxDriveAcceleration(this.driveMotor, WHEEL_DIAMETER, this.robotMass)
            .times(scaleFactor);

    this.maxSteeringSpeed =
        this.moduleType.calculateMaxSteeringSpeed(this.steeringMotor).times(scaleFactor);
    this.maxSteeringAcceleration =
        this.moduleType
            .calculateMaxSteeringAcceleration(this.steeringMotor, this.robotMass)
            .times(scaleFactor);

    final Distance wheelTrackRadius =
        Units.Meters.of(Math.hypot(this.wheelDistanceX, this.wheelDistanceY));

    this.maxRotationalSpeed =
        Units.RadiansPerSecond.of(
            this.maxDriveSpeed.in(Units.MetersPerSecond) / wheelTrackRadius.in(Units.Meters));
    this.maxRotationalAcceleration =
        Units.RadiansPerSecondPerSecond.of(
            this.maxDriveAcceleration.in(Units.MetersPerSecondPerSecond)
                / wheelTrackRadius.in(Units.Meters));

    this.wheelPositions =
        new Translation2d[] {
          new Translation2d(this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
        };

    this.kinematics = new SwerveDriveKinematics(wheelPositions);
    this.kinematicsConstraint =
        new SwerveDriveKinematicsConstraint(
            kinematics, this.maxDriveSpeed.in(Units.MetersPerSecond));
    this.steeringConstraints =
        new TrapezoidProfile.Constraints(
            this.maxSteeringSpeed.in(Units.RadiansPerSecond),
            this.maxSteeringAcceleration.in(Units.RadiansPerSecondPerSecond));
    this.robotRotationalConstraints =
        new TrapezoidProfile.Constraints(
            this.maxRotationalSpeed.in(Units.RadiansPerSecond),
            this.maxRotationalAcceleration.in(Units.RadiansPerSecondPerSecond));
  }

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg including bumpers and battery.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters.
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param driveFeedforward The drive feedforward constants.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private RobotParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleType swerveModule,
      Motors driveMotor,
      Motors steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      FeedforwardConstants driveFeedForward,
      int pigeonID) {
    this(
        robotMass,
        wheelDistanceX,
        wheelDistanceY,
        swerveModule,
        driveMotor,
        steeringMotor,
        motorIds,
        angleEncoderIds,
        angleOffset,
        driveFeedForward,
        new CalculatedFeedforwardConstants(
            steeringMotor.getKs(),
            () -> swerveModule.calculateMaxSteeringSpeed(steeringMotor).in(Units.RadiansPerSecond),
            () ->
                swerveModule
                    .calculateMaxSteeringAcceleration(steeringMotor, robotMass)
                    .in(Units.RadiansPerSecondPerSecond)),
        pigeonID);
  }

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private RobotParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleType swerveModule,
      Motors driveMotor,
      Motors steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      int pigeonID) {
    this(
        robotMass,
        wheelDistanceX,
        wheelDistanceY,
        swerveModule,
        driveMotor,
        steeringMotor,
        motorIds,
        angleEncoderIds,
        angleOffset,
        new CalculatedFeedforwardConstants(
            driveMotor.getKs(),
            () ->
                swerveModule
                    .calculateMaxDriveSpeed(driveMotor, WHEEL_DIAMETER)
                    .in(Units.MetersPerSecond),
            () ->
                swerveModule
                    .calculateMaxDriveAcceleration(driveMotor, WHEEL_DIAMETER, robotMass)
                    .in(Units.MetersPerSecondPerSecond)),
        new CalculatedFeedforwardConstants(
            steeringMotor.getKs(),
            () -> swerveModule.calculateMaxSteeringSpeed(steeringMotor).in(Units.RadiansPerSecond),
            () ->
                swerveModule
                    .calculateMaxSteeringAcceleration(steeringMotor, robotMass)
                    .in(Units.DegreesPerSecondPerSecond)),
        pigeonID);
  }

  /**
   * Returns the mass of the robot in Kg.
   *
   * @return The mass of the robot in Kg.
   */
  public double getRobotMass() {
    return this.robotMass;
  }

  /**
   * Returns the maximum rotational speed of the robot in rad/s.
   *
   * @return The maximum rotation speed of the robot in rad/s.
   */
  public AngularVelocity getMaxRotationalSpeed() {
    return this.maxRotationalSpeed;
  }

  /**
   * Returns the maximum rotational acceleration of the robot in rad/s^2.
   *
   * @return The maximum rotation acceleration of the robot in rad/s^s.
   */
  public AngularAcceleration getMaxRotationalAcceleration() {
    return this.maxRotationalAcceleration;
  }

  /**
   * Return the wheel base radius in meters.
   *
   * @return The wheel base radius in meters.
   */
  public double getWheelBaseRadius() {
    double halfWheelDistanceX = wheelDistanceX / 2;
    double halfWheelDistanceY = wheelDistanceY / 2;

    return Math.sqrt(
        halfWheelDistanceX * halfWheelDistanceX + halfWheelDistanceY * halfWheelDistanceY);
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfilePIDController} used to reach the goal robot orientation.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal robot orientation.
   */
  public TrapezoidProfile.Constraints getRotationalConstraints() {
    return this.robotRotationalConstraints;
  }

  /**
   * Returns the distance between the wheels along the X-axis in meters.
   *
   * @return The distance between the wheels along the X-axis in meters.
   */
  public double getWheelDistanceX() {
    return this.wheelDistanceX;
  }

  /**
   * Returns the distance between the wheels along the Y-axis in meters.
   *
   * @return The distance between the wheels along the Y-axis in meters.
   */
  public double getWheelDistanceY() {
    return this.wheelDistanceY;
  }

  /**
   * Returns the wheel positions relative to the robot center.
   *
   * @return The wheel positions relative to the robot center.
   */
  public Translation2d[] getWheelPositions() {
    return this.wheelPositions;
  }

  /**
   * Returns the wheel diameter in meters.
   *
   * @return The wheel diameter in meters.
   */
  public double getWheelDiameter() {
    return WHEEL_DIAMETER.in(Units.Meters);
  }

  /**
   * Returns the drive gear ratio.
   *
   * @return The drive gear ratio.
   */
  public double getDriveGearRatio() {
    return this.moduleType.getDriveGearRatio();
  }

  /**
   * Returns the steering gear ratio.
   *
   * @return The steering gear ratio.
   */
  public double getSteeringGearRatio() {
    return this.moduleType.getSteeringGearRatio();
  }

  /**
   * Returns the swerve module used by the robot.
   *
   * @return The swerve module used by the robot.
   */
  public SwerveModuleType getModuleType() {
    return this.moduleType;
  }

  /**
   * Returns the motor used by swerve module on the robot.
   *
   * @return The motor used by swerve module on the robot.
   */
  public Motors getMotors() {
    return this.driveMotor;
  }

  /**
   * Returns the CAN ids of the specified motor.
   *
   * @param motor The motor.
   * @return The CAN id.
   */
  public int getMotorId(SwerveMotors motor) {
    return this.motorIds[motor.getIndex()];
  }

  /**
   * Returns the motor controller for the specified motor.
   *
   * @param motor The motor.
   * @return The motor controller.
   */
  public MotorController getMotorController(SwerveMotors motor) {
    String logPrefix = "/SwerveModule/" + motor.name();
    int motorID = getMotorId(motor);
    MotorConfig motorConfig;

    switch (motor) {
      case FrontLeftDrive:
      case FrontRightDrive:
      case BackLeftDrive:
      case BackRightDrive:
        double metersPerRotation = (getWheelDiameter() * Math.PI) / getDriveGearRatio();
        motorConfig = new MotorConfig(COUNTER_CLOCKWISE_POSITIVE, BRAKE, metersPerRotation);

        return driveMotor.newController(logPrefix, motorID, motorConfig, DRIVE_CURRENT_CONFIG);

      default:
        double radiansPerRotation = (2 * Math.PI) / getSteeringGearRatio();
        motorConfig = new MotorConfig(getSteeringDirection(), BRAKE, radiansPerRotation);

        return steeringMotor.newController(
            logPrefix, motorID, motorConfig, STEERING_CURRENT_CONFIG);
    }
  }

  /**
   * Returns the CANcoder ids of the specified module.
   *
   * @param angleEncoder The angle encoder.
   * @return The CANcoder id.
   */
  public int getAngleEncoderId(SwerveAngleEncoder angleEncoder) {
    return this.angleEncoderIds[angleEncoder.getIndex()];
  }

  /**
   * Returns the angle offsets for the specified module.
   *
   * @return the angle offsets for the specified module.
   */
  public double getAngleOffset(SwerveAngleEncoder angleEncoder) {
    return this.angleOffset[angleEncoder.getIndex()];
  }

  /**
   * Returns and configures the angle encoder for the specified module.
   *
   * @param angleEncoder The angle encoder.
   * @return The angle encoder for the specified module.
   */
  public CANcoder getAngleEncoder(SwerveAngleEncoder angleEncoder) {
    int deviceID = getAngleEncoderId(angleEncoder);
    CANcoder wheelAngleEncoder = new CANcoder(deviceID);
    CANcoderConfigurator wheelAngleConfigurator = wheelAngleEncoder.getConfigurator();
    CANcoderConfiguration wheelAngleConfig = new CANcoderConfiguration();

    wheelAngleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    wheelAngleConfig.MagnetSensor.MagnetOffset = getAngleOffset(angleEncoder) / 360.0;
    wheelAngleConfigurator.apply(wheelAngleConfig);
    return wheelAngleEncoder;
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   *
   * @return The maximum drive speed.
   */
  public LinearVelocity getMaxDriveSpeed() {
    return this.maxDriveSpeed;
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   *
   * @return The maximum drive acceleration.
   */
  public LinearAcceleration getMaxDriveAcceleration() {
    return this.maxDriveAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for translation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for translation in Volts.
   */
  public double getDriveKs() {
    return this.driveFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for translation in Volt * seconds per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per meter.
   */
  public double getDriveKv() {
    return this.driveFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for translation in Volt * seconds^2 per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per meter.
   */
  public double getDriveKa() {
    return this.driveFeedforward.kA;
  }

  /**
   * Returns a {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   * module states.
   *
   * @return A {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   *     module states.
   */
  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   * kinematics constraints when following a trajectory.
   *
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   *     kinematics constraints when following a trajectory.
   */
  public SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return this.kinematicsConstraint;
  }

  /**
   * Returns the maximum steering speed in rad/s of a swerve module.
   *
   * @return The maximum steering speed.
   */
  public AngularVelocity getMaxSteeringSpeed() {
    return this.maxSteeringSpeed;
  }

  /**
   * Returns the maximum steering acceleration in rad/s^2 of a swerve module.
   *
   * @return The maximum steering acceleration.
   */
  public AngularAcceleration getMaxSteeringAcceleration() {
    return this.maxSteeringAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for rotation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for rotation in Volts.
   */
  public double getSteeringKs() {
    return this.steeringFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for rotation in Volt * seconds per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant steering velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per radian.
   */
  public double getSteeringKv() {
    return this.steeringFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for rotation in Volt * seconds^2 per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant rotational acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per radian.
   */
  public double getSteeringKa() {
    return this.steeringFeedforward.kA;
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfiledPIDController} used to reach the goal wheel angle.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal wheel angle.
   */
  public TrapezoidProfile.Constraints getSteeringConstraints() {
    return steeringConstraints;
  }

  /**
   * Returns the direction the steering motor must rotate when a positive voltage is applied to
   * rotate the wheel in the counter-clockwise direction.
   *
   * @return The direction the steering motor must rotate when a positive voltage is applied to
   *     rotate the wheel in the counter-clockwise direction.
   */
  public MotorDirection getSteeringDirection() {
    return moduleType.getSteeringDirection();
  }

  /** Returns the correct gyro implementation for the robot. */
  public Gyro getGyro() {
    return new Pigeon2Gyro(pigeonID);
  }

  /** Returns the moment of the robot base. */
  public double getMomentOfInertia() {
    return getRobotMass()
        * (Math.pow(getWheelDistanceX(), 2) + Math.pow(getWheelDistanceY(), 2))
        / 12;
  }

  /** Returns the robot config for pathplanner. */
  public RobotConfig getPathplannerConfig() {
    ModuleConfig moduleconfig =
        new ModuleConfig(
            getWheelDiameter() / 2,
            getMaxDriveSpeed().in(Units.MetersPerSecond),
            1.0,
            driveMotor.getDCMotor(),
            getDriveGearRatio(),
            140,
            1);
    return new RobotConfig(getRobotMass(), getMomentOfInertia(), moduleconfig, this.wheelPositions);
  }
}
