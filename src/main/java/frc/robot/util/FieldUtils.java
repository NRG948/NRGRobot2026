package frc.robot.util;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.parameters.AprilTagFieldParameters;


public final class FieldUtils {
    private static final int RED_HUB_APRILTAG = 10;
    private static final int BLUE_HUB_APRILTAG = 26;

    @RobotPreferencesValue
  public static RobotPreferences.EnumValue<AprilTagFieldParameters> FIELD_LAYOUT_PREFERENCE =
      new RobotPreferences.EnumValue<AprilTagFieldParameters>(
          "AprilTag", "Field Layout", AprilTagFieldParameters.k2026RebuiltWelded);

  private static AprilTagFieldLayout FIELD_LAYOUT =
      FIELD_LAYOUT_PREFERENCE.getValue().loadAprilTagFieldLayout();

    
    public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red;
  }

  /** Returns the {@link Pose3d} of the specified April Tag ID. */
  public static Pose3d getAprilTagPose3d(int tagId) {
    return FIELD_LAYOUT.getTagPose(tagId).get();
  }

  /** Returns the {@link Pose2d} of the specified April Tag ID. */
  public static Pose2d getAprilTagPose2d(int tagId) {
    return getAprilTagPose3d(tagId).toPose2d();
  }

    private static int getHubAprilTagID() {
        if (isRedAlliance()) {
            return RED_HUB_APRILTAG;
        }
        return BLUE_HUB_APRILTAG;
    }
    /**
     * 
     * @return Pose2d of allicance-side hub's center April tag
     */
    public static Pose2d getHubAprilTag() {
        return getAprilTagPose2d(getHubAprilTagID());
    }

    public static AprilTagFieldLayout getFieldLayout() {
    return FIELD_LAYOUT;
  }
}
