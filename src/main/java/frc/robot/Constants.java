// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    public static final Pose2d testPose = new Pose2d(10.33, 1.18, new Rotation2d());
    //TODO: GOAL END STATE SETS THE HEADING FOR THE HOLONOMIC ROTATION, YOU WIL LHAVE TO FLIP FOR ALLIANCE COLOR
    private static final Pose2d waypoint1 = new Pose2d(6.5, 3.98, new Rotation2d(Math.PI));
    private static final Pose2d waypoint2 = new Pose2d(5.4, 3.98, new Rotation2d(Math.PI));
    private static List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        waypoint1,
        waypoint2
);

    public static final PathPlannerPath testPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, new Rotation2d(Math.PI)));

    public static final Pose3d algaeArmOffset = new Pose3d(0, -0.305, 0.937, new Rotation3d(0, 0, 0));
    public static final Pose3d algaeArmRollerOffset = new Pose3d(0, -0.102, 0.725, new Rotation3d(0, 0, 0));
    public static final Pose3d climberOffset = new Pose3d(0.311, 0, 0.273, new Rotation3d(0, 0, 0));
    public static final Pose3d coralIntakeOffset = new Pose3d(0, 0.254, 0.210, new Rotation3d(0, 0, 0));
    public static final Pose3d coralIntakeRollerOffset = new Pose3d(0, 0.4, 0.738, new Rotation3d(0, 0, 0));
    public static final Pose3d elevatorInnerOffset = new Pose3d(0, -0.247, 0.081, new Rotation3d(0, 0, 0));
    public static final Pose3d elevatorOuterOffset = new Pose3d(0, -0.3267, 0.070, new Rotation3d(0, 0, 0));

    public static final ArrayList<Pose2d> drivingPoses = new ArrayList<Pose2d>(Arrays.asList(reefBranchA, humanPlayerStationRight, reefBranchH));

    public static final int goalVelocity = 0;

    public static final Pose3d scorerPoseOffset = new Pose3d(0, -0.305, 0.937, new Rotation3d(0, 0, 0));
    public static final Pose3d scorerRollerPoseOffset = new Pose3d(0, -0.305, 0.937, new Rotation3d(0, 0, 0));//0, -0.102, 0.725
    public static final Pose3d climberPoseOffset = new Pose3d(0.311, 0, 0.273, new Rotation3d(0, 0, 0));
    public static final Pose3d coralIntakePoseOffset =  new Pose3d(0, 0.254, 0.210, new Rotation3d(0, 0, 0));
    public static final Pose3d coralIntakeRollerPoseOffset = new Pose3d(0, 0.4, 0.738, new Rotation3d(0, 0, 0));
    public static final Pose3d elevatorOneIntakeOffset = new Pose3d(0, -0.247, 0.081, new Rotation3d(0, 0, 0));
    public static final Pose3d elevatorTwoIntakeOffset = new Pose3d(0, -0.3267, 0.070, new Rotation3d(0, 0, 0));

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
