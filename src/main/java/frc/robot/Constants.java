// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
    

    //Constants to use for pathfinding
    public static final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    public static final Pose2d targetPose = new Pose2d(5.8, 4, new Rotation2d(Math.PI));
    public static final Pose2d humanPlayerStationRight = new Pose2d(1.67, 0.65, new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d humanPlayerStationLeft = new Pose2d(1.62, 7.36, new Rotation2d(Units.degreesToRadians(-54)));
    public static final Pose2d reefBranchA = new Pose2d(3.27, 4.16, new Rotation2d());
    public static final Pose2d reefBranchB = new Pose2d(3.27, 3.85, new Rotation2d());
    public static final Pose2d reefBranchC = new Pose2d(3.75, 3.05, new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d reefBranchD = new Pose2d(4.0, 2.82, new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d reefBranchE = new Pose2d(4.94, 2.89, new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d reefBranchF = new Pose2d(5.31, 3.02, new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d reefBranchG = new Pose2d(5.7, 3.86, new Rotation2d(Math.PI));
    public static final Pose2d reefBranchH = new Pose2d(5.7, 4.2, new Rotation2d(Math.PI));
    public static final Pose2d reefBranchI = new Pose2d(5.26, 5.07, new Rotation2d(Units.degreesToRadians(-120)));
    public static final Pose2d reefBranchJ = new Pose2d(5.0, 5.21, new Rotation2d(Units.degreesToRadians(-120)));
    public static final Pose2d reefBranchK = new Pose2d(4.0, 5.22, new Rotation2d(Units.degreesToRadians(-60)));
    public static final Pose2d reefBranchL = new Pose2d(3.72, 5.04, new Rotation2d(Units.degreesToRadians(-60)));

    public static final int goalVelocity = 1;
    public static final int speedMultiplier = 3;




    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }


}
