/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.preferences.BooleanPreference;
import com.nrg948.util.ReflectionUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.lang.reflect.InvocationTargetException;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Utility class for subsystem-related operations. */
public final class SubsystemsUtil {
  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The type of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @param initArgs The arguments to pass to the subsystem's constructor.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, BooleanPreference enabled, Object... initArgs) {
    if (!enabled.getValue()) {
      return Optional.empty();
    }

    Class<?>[] initArgClasses =
        Stream.of(initArgs)
            .map(Object::getClass)
            .map(ReflectionUtil::unbox)
            .toArray(Class<?>[]::new);

    try {
      return Optional.of(subsystemClass.getConstructor(initArgClasses).newInstance(initArgs));
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
      e.printStackTrace();
      return Optional.empty();
    } catch (NoSuchMethodException e) {
      System.err.printf(
          "ERROR: The class is missing constructor %s(%s).%n",
          subsystemClass.getName(),
          Stream.of(initArgClasses).map(Class::getSimpleName).collect(Collectors.joining(", ")));
      e.printStackTrace();
      return Optional.empty();
    }
  }

  private SubsystemsUtil() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
}
