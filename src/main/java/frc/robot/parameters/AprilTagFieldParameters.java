/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import com.nrg948.preferences.EnumPreference;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * AprilTag field layout selections for REBUILT.
 *
 * <p>The AprilTag location may vary depending on the field perimeter in use.
 *
 * <p>This enum can be used with a {@link SendableChooser} or {@link EnumPreference} to select the
 * field layout to match.
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
