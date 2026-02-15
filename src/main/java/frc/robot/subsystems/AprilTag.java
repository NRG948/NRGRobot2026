/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardCameraStream;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardLayout;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.preferences.EnumPreference;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import frc.robot.RobotSelector;
import frc.robot.parameters.PoseEstimationStrategy;
import frc.robot.util.FieldUtils;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@DashboardDefinition
public class AprilTag extends SubsystemBase {

  private static final DataLog LOG = DataLogManager.getLog();

  private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  private static final PhotonPipelineResult NO_RESULT = new PhotonPipelineResult();
  private static final double LAST_RESULT_TIMEOUT = 0.1;

  // TODO: measure ALL camera rotations and transforms for the 2026 robot.
  private static final Rotation3d FRONT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-16), 0);
  public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA =
      new Transform3d(new Translation3d(+0.105, +0.259, +0.668), FRONT_CAMERA_ROTATION);
  public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA =
      new Transform3d(new Translation3d(+0.105, -0.262, +0.668), FRONT_CAMERA_ROTATION);

  /**
   * A single camera's parameters.
   *
   * @param cameraName The name of the camera.
   * @param robotToCamera The transform from the robot's odometry center to the camera.
   * @param cameraPublisherName The camera publisher of the camera.
   * @param streamURL The stream URL of the camera.
   */
  public record CameraParameters(
      String cameraName, Transform3d robotToCamera, String cameraPublisherName, String streamURL) {}

  /**
   * The robot's vision parameters.
   *
   * @param frontRight The front right camera parameters.
   * @param frontLeft The front left camera parameters.
   * @param backRight The back right camera parameters.
   * @param backLeft The back left camera parameters.
   */
  public record VisionParameters(
      Optional<CameraParameters> frontLeft,
      Optional<CameraParameters> frontRight,
      Optional<CameraParameters> backLeft,
      Optional<CameraParameters> backRight) {}

  // TODO: Find streamPorts for cameras on practice robot
  public static final VisionParameters PRACTICE_VISION_PARAMS =
      new VisionParameters(
          Optional.of(
              new CameraParameters(
                  "FrontLeftCamera",
                  ROBOT_TO_FRONT_LEFT_CAMERA,
                  "photonvision_Port_1182_Output_MJPEG_Server",
                  "http://photonvision.local:1182/stream.mjpg")), // Port: 1182
          Optional.of(
              new CameraParameters(
                  "FrontRightCamera",
                  ROBOT_TO_FRONT_RIGHT_CAMERA,
                  "photonvision_Port_1184_Output_MJPEG_Server",
                  "http://photonvision2.local:1184/stream.mjpg")), // Port: 1184
          Optional.empty(),
          Optional.empty());
  public static final VisionParameters COMPETITION_VISION_PARAMS =
      new VisionParameters(Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty());
  public static final VisionParameters ALPHA_VISION_PARAMS =
      new VisionParameters(Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty());

  public static final VisionParameters PARAMETERS =
      RobotPreferences.ROBOT_TYPE
          .select(
              Map.of(
                  RobotSelector.PracticeRobot2026, PRACTICE_VISION_PARAMS,
                  RobotSelector.CompetitionRobot2026, COMPETITION_VISION_PARAMS,
                  RobotSelector.AlphaRobot2026, ALPHA_VISION_PARAMS))
          .orElse(COMPETITION_VISION_PARAMS);

  public static EnumPreference<PoseEstimationStrategy> POSE_ESTIMATION_STRATEGY =
      new EnumPreference<PoseEstimationStrategy>(
          "AprilTag", "Pose Est. Strategy", PoseEstimationStrategy.MultiTagPnpOnCoprocessor);

  private final PhotonCamera camera;
  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator estimator;

  @DashboardComboBoxChooser(column = 0, row = 0, width = 2, title = "Selected April Tag")
  private final SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<>();

  private final BooleanLogEntry hasTargetLogger;
  private final StructLogEntry<Pose2d> estimatedPoseLogger;
  private final StructArrayLogEntry<Pose2d> targetPoseArrayLogger;

  private Optional<PhotonPipelineResult> result = Optional.empty();

  private int selectedAprilTag;
  private Pose3d selectedAprilTagPose = new Pose3d();

  @DashboardLayout(title = "Selected April Tag", column = 9, row = 0, width = 2, height = 5)
  private SelectedAprilTagTelemetry selectedAprilTagTelemetry = new SelectedAprilTagTelemetry();

  @DashboardLayout(title = "Estimated Pose", column = 0, row = 3, width = 2, height = 2)
  private EstimatedPoseTelemetry estimatedPoseTelemetry = new EstimatedPoseTelemetry();

  private Optional<EstimatedRobotPose> globalEstimatedPose = Optional.empty();

  private Pose2d lastEstimatedPose = Pose2d.kZero;

  private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS;

  @DashboardCameraStream(title = "Camera Stream", column = 2, row = 0, width = 4, height = 4)
  private HttpCamera video;

  /**
   * Constructs a new AprilTagSubsystem instance.
   *
   * @param cameraName The name of the camera.
   * @param robotToCamera The transform from the robot to the camera.
   * @param cameraPublisherName The camera publisher of the camera.
   * @param streamURL The stream URL of the camera.
   */
  public AprilTag(
      String cameraName, Transform3d robotToCamera, String cameraPublisherName, String streamURL) {
    setName(cameraName);
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.cameraToRobot = robotToCamera.inverse();

    estimator = new PhotonPoseEstimator(FieldUtils.getFieldLayout(), robotToCamera);

    for (int i = 1; i <= 32; i++) {
      aprilTagIdChooser.addOption(String.valueOf(i), i);
    }
    aprilTagIdChooser.setDefaultOption("1", 1);

    hasTargetLogger = new BooleanLogEntry(LOG, String.format("/%s/Has Target", cameraName));
    estimatedPoseLogger =
        StructLogEntry.create(LOG, String.format("/%s/Estimated Pose", cameraName), Pose2d.struct);
    targetPoseArrayLogger =
        StructArrayLogEntry.create(
            LOG, String.format("/%s/Target Poses", cameraName), Pose2d.struct);
    video =
        new HttpCamera(
            String.format(cameraPublisherName),
            String.format(streamURL),
            HttpCameraKind.kMJPGStreamer);
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return this.globalEstimatedPose;
  }

  /**
   * Calculates new standard deviations for pose estimation.
   *
   * <p>This algorithm is a heuristic that creates dynamic standard deviations based on number of
   * tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = SINGLE_TAG_STD_DEVS;
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = SINGLE_TAG_STD_DEVS;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = SINGLE_TAG_STD_DEVS;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple tags are visible
        if (numTags > 1) {
          estStdDevs = MULTI_TAG_STD_DEVS;
        }
        // Increase std devs based on (average) distance.
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * Estimates the robot pose based on the selected robot pose estimation strategy.
   *
   * @param result Pipeline result from the camera.
   * @return Estimated pose.
   */
  public Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult result) {
    switch (RobotPreferences.POSE_ESTIMATION_STRATEGY.getValue()) {
      case AverageBestTargets:
        return estimator.estimateAverageBestTargetsPose(result);
      case ClosestToCameraHeight:
        return estimator.estimateClosestToCameraHeightPose(result);
      case MultiTagPnpOnCoprocessor:
        var res = estimator.estimateCoprocMultiTagPose(result);
        if (res.isPresent()) {
          return res;
        }
      // Fallthrough
      case LowestAmbiguity:
        return estimator.estimateLowestAmbiguityPose(result);
      case PnpDistanceTrigSolve:
        return estimator.estimatePnpDistanceTrigSolvePose(result);
      default:
        System.out.println(
            "ERROR: Unsupported pose estimation strategy: "
                + RobotPreferences.POSE_ESTIMATION_STRATEGY.getValue().name());
        return Optional.empty();
    }
  }

  @Override
  public void periodic() {
    // Process the latest vision results updating the estimated robot pose and
    // current result.
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    Optional<PhotonPipelineResult> currentResult = Optional.empty();
    List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();
    for (var change : allUnreadResults) {
      Optional<EstimatedRobotPose> visionEstTemp = estimateRobotPose(change);

      // Only update the vision estimate if it is not empty.
      // This way, we discard empty updates from the coprocessor
      // and ensure we are always receiving fresh data every control cycle.
      if (visionEstTemp.isEmpty()) {
        continue;
      }
      visionEst = visionEstTemp;
      updateEstimationStdDevs(visionEst, change.getTargets());
      currentResult = Optional.of(change);
    }

    globalEstimatedPose = visionEst;
    globalEstimatedPose.ifPresent(
        (e) -> {
          lastEstimatedPose = e.estimatedPose.toPose2d();
          estimatedPoseLogger.append(lastEstimatedPose);
        });

    // Update the result if present or if the last result is within the lifetime
    // timeout period.
    if (currentResult.isPresent()
        || this.result
            .map(r -> (Timer.getFPGATimestamp() - r.getTimestampSeconds()) < LAST_RESULT_TIMEOUT)
            .orElse(false)) {
      this.result = currentResult;

      // Log the visible target poses.
      Pose2d[] targets =
          currentResult.orElse(NO_RESULT).getTargets().stream()
              .map(t -> FieldUtils.getAprilTagPose2d(t.getFiducialId()))
              .toArray(Pose2d[]::new);
      targetPoseArrayLogger.append(targets);
    }

    hasTargetLogger.update(hasTargets());

    selectedAprilTag = aprilTagIdChooser.getSelected().intValue();
    selectedAprilTagPose = FieldUtils.getAprilTagPose3d(selectedAprilTag);
    selectedAprilTagTelemetry.selectedAprilTagPoseX = selectedAprilTagPose.getX();
    selectedAprilTagTelemetry.selectedAprilTagPoseY = selectedAprilTagPose.getY();
    selectedAprilTagTelemetry.selectedAprilTagPoseZ = selectedAprilTagPose.getZ();
    selectedAprilTagTelemetry.selectedAprilTagRoll = selectedAprilTagPose.getRotation().getX();
    selectedAprilTagTelemetry.selectedAprilTagPitch = selectedAprilTagPose.getRotation().getY();
    selectedAprilTagTelemetry.selectedAprilTagYaw = selectedAprilTagPose.getRotation().getZ();

    estimatedPoseTelemetry.lastEstimatedPoseX = lastEstimatedPose.getX();
    estimatedPoseTelemetry.lastEstimatedPoseY = lastEstimatedPose.getY();
    estimatedPoseTelemetry.lastEstimatedPoseYaw = lastEstimatedPose.getRotation().getDegrees();

    Optional<PhotonTrackedTarget> target = getTarget(selectedAprilTag);
    if (target.isPresent()) {
      var robotToTarget = robotToCamera.plus(target.get().getBestCameraToTarget());
      selectedAprilTagTelemetry.distanceToSelectedTarget =
          Math.hypot(robotToTarget.getX(), robotToTarget.getY());
      selectedAprilTagTelemetry.angleToSelectedTarget =
          Math.atan2(robotToTarget.getY(), robotToTarget.getX());
    }
  }

  /**
   * Returns the latest vision result
   *
   * @return latest vision result
   */
  protected PhotonPipelineResult getLatestResult() {
    return result.orElse(NO_RESULT);
  }

  /**
   * Return the transform from the camera to the center of the robot
   *
   * @return Return the transform from the camera to the center of the robot
   */
  public Transform3d getCameraToRobotTransform() {
    return cameraToRobot;
  }

  /***
   * Return the transform from the center of the robot to camera
   *
   * @return Return the transform from center of the robot to camera
   */
  public Transform3d getRobotToCameraTransform() {
    return robotToCamera;
  }

  /** Returns whether one or more AprilTags are visible. */
  @DashboardBooleanBox(column = 0, row = 1, width = 2, height = 2, title = "Has Targets")
  public boolean hasTargets() {
    return result.orElse(NO_RESULT).hasTargets();
  }

  /** Returns the best target AprilTag. */
  public PhotonTrackedTarget getBestTarget() {
    return result.orElse(NO_RESULT).getBestTarget();
  }

  /**
   * Returns the timestamp of the latest vision result. This is only valid if the subsystem has
   * targets.
   */
  public double getTargetTimeStamp() {
    return result.orElse(NO_RESULT).getTimestampSeconds();
  }

  /** Returns the visible AprilTag targets. */
  public List<PhotonTrackedTarget> getTargets() {
    return result.orElse(NO_RESULT).getTargets();
  }

  /**
   * Returns the AprilTag target of the input ID.
   *
   * @param id The AprilTag ID.
   * @return The target with the input ID.
   */
  public Optional<PhotonTrackedTarget> getTarget(int id) {
    return getTargets().stream().filter(target -> target.getFiducialId() == id).findFirst();
  }

  /**
   * Returns the distance from center of the robot to the target with the input ID. Returns 0 if
   * target not found.
   *
   * @param id The AprilTag ID.
   * @return The distance to the target with the input ID.
   */
  public double getDistanceToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    var bestCameraToTarget = robotToCamera.plus(target.get().getBestCameraToTarget());
    return Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
  }

  @DashboardDefinition
  public class SelectedAprilTagTelemetry {
    @DashboardTextDisplay(column = 0, row = 0, title = "X")
    public double selectedAprilTagPoseX;

    @DashboardTextDisplay(column = 1, row = 0, title = "Y")
    public double selectedAprilTagPoseY;

    @DashboardTextDisplay(column = 2, row = 0, title = "Z")
    public double selectedAprilTagPoseZ;

    @DashboardTextDisplay(column = 0, row = 1, title = "Roll")
    public double selectedAprilTagRoll;

    @DashboardTextDisplay(column = 1, row = 1, title = "Pitch")
    public double selectedAprilTagPitch;

    @DashboardTextDisplay(column = 2, row = 1, title = "Yaw")
    public double selectedAprilTagYaw;

    @DashboardTextDisplay(column = 0, row = 2, title = "Angle to Target")
    public double angleToSelectedTarget;

    @DashboardTextDisplay(column = 1, row = 2, title = "Distance to Target")
    public double distanceToSelectedTarget;
  }

  @DashboardDefinition
  public class EstimatedPoseTelemetry {
    @DashboardTextDisplay(column = 0, row = 0, title = "X")
    public double lastEstimatedPoseX;

    @DashboardTextDisplay(column = 1, row = 0, title = "Y")
    public double lastEstimatedPoseY;

    @DashboardTextDisplay(column = 2, row = 0, title = "Yaw")
    public double lastEstimatedPoseYaw;
  }
}
