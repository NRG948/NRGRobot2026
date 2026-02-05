/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * AprilTag field layout selections for REEFSCAPE.
 *
 * <p>The PROCESSOR AprilTag location varies depending on the field perimeter in use. See <a
 * href="https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate12.pdf">Team
 * Update 12</a> for more information.
 *
 * <p>This enum can be used with a {@link SendableChooser} or {@link RobotPreferences.EnumValue} to
 * select the field layout to match.
 */
public enum AprilTagFieldParameters {

  /** The field layout using the welded perimeter. */
  k2026RebuiltWelded(AprilTagFields.k2026RebuiltWelded),

  /** The field layout using the AndyMark perimeter. */
  k2026RebuiltAndymark(AprilTagFields.k2026RebuiltAndymark);

  private AprilTagFields aprilTagField;

  /** Constructs a variant of this enum. */
  private AprilTagFieldParameters(AprilTagFields aprilTagField) {
    this.aprilTagField = aprilTagField;
  }

  /** Gets the {@link AprilTagFields} variant that can be used to load the correct field layout. */
  public AprilTagFields getAprilTagField() {
    return aprilTagField;
  }

  /** Gets the {@link AprilTagFieldLayout} for this variant. */
  public AprilTagFieldLayout loadAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(aprilTagField);
  }
}
