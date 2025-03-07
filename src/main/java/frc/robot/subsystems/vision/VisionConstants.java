// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "OV2311FrontRightCamera";
    public static String camera1Name = "OV2331FrontLeftCamera";
    public static String camera2Name = "OV2311BackRightCamera";
    public static String camera3Name = "OV2311BackLeftCamera";
    public static String camera4Name = "OV2311UpperRightCamera";
    public static String camera5Name = "OV2311UpperLeftCamera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(0.34, -0.279, 0.187, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-39.4157768)));
    public static Transform3d robotToCamera1 = new Transform3d(0.34, 0.279, 0.187, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(39.4157768)));
    public static Transform3d robotToCamera2 = new Transform3d(-0.34, -0.279, 0.187, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-140.5842232)));
    public static Transform3d robotToCamera3 = new Transform3d(-0.34, 0.279, 0.187, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(140.5842232)));
    public static Transform3d robotToCamera4 = new Transform3d(0.185, -0.082, 0.242, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(25)));
    public static Transform3d robotToCamera5 = new Transform3d(0.185, 0.082, 0.242, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-25)));

    // Maximo stuff :)
    public static double UpperLeftCameraPerfectXOffset = 0.2490851;

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.05; // Meters
    public static double angularStdDevBaseline = 0.015; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
