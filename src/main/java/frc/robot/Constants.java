// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.64135 / 2.0, 0.64135 / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(0.64135 / 2.0, -0.64135 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.64135 / 2.0, 0.64135 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.64135 / 2.0, -0.64135 / 2.0);

    public static final double maxModuleSpeed = 4.5; // M/S

  }

  public static class VisionConstants {
    public final static boolean USE_VISION = true;

    // public static final Transform3d ROBOT_TO_PINKY = new Transform3d(
    //     new Translation3d(-.32, 0.28, 0.35),
    //     new Rotation3d(0, Math.toRadians(-13), Math.toRadians(165)));

    public static final Transform3d ROBOT_TO_STINKY = new Transform3d(
        new Translation3d(-.2794, 0, 0.3556),
        new Rotation3d(0, Math.toRadians(-13), Math.toRadians(0)));
  }

  public static class FieldConstants {
    public static final double LENGTH = Units.feetToMeters(54);
    public static final double WIDTH = Units.feetToMeters(27);
  }
}
