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

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.CameraTargetRelation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.trees.Targets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.AllianceFlipUtil;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    public static double CenterXDistance;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
                    AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        debugxOffset();

    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    // public double CenterAngleToAprilTag() {
    // double PitchAngle = inputs[3].latestTargetObservationDouble.ty();
    // double YawAngle = inputs[3].latestTargetObservationDouble.tx();

    // double xDist = 0.0624 * Math.cos(YawAngle) / Math.tan(PitchAngle);
    // return Math.atan((xDist * Math.tan(YawAngle) - 0.082) / (xDist + 0.185));
    // }

    public double cameraDistanceToAprilTag(Blackboard blackboard) {
        double pitchAngle;
        if (blackboard.get("target") != Targets.NONE) {
            if (blackboard.isTargetLeftBranch("target")) {
                pitchAngle = inputs[3].latestTargetObservationDouble.ty();
            } else {
                pitchAngle = inputs[2].latestTargetObservationDouble.ty();
            }
        } else {
            pitchAngle = inputs[findClosestCamera(blackboard)].latestTargetObservationDouble.ty();
        }
        pitchAngle = Units.degreesToRadians(pitchAngle);

        double distance = .0624 / Math.tan(pitchAngle);
        return distance;
    }

    public double robotXOffsetToAprilTag(Blackboard blackboard) {
        double distance = cameraDistanceToAprilTag(blackboard);
        double yawAngle;
        double flipYawAngle;
        int cameraIndex = findClosestCamera(blackboard);
        if (blackboard.getTarget("target") != Targets.NONE) {
            io[2].getTargetID(blackboard);
            io[3].getTargetID(blackboard);

            if (blackboard.isTargetLeftBranch("target")) {
                yawAngle = inputs[3].latestTargetObservationDouble.tx();
                flipYawAngle = 1.0;
            } else {
                yawAngle = inputs[2].latestTargetObservationDouble.tx();
                flipYawAngle = -1.0;
            }
        } else {
            yawAngle = inputs[cameraIndex].latestTargetObservationDouble.tx();
            if (cameraIndex == 3) {
                flipYawAngle = 1.0;
            } else {
                flipYawAngle = -1.0;
            }
        }
        yawAngle = Units.degreesToRadians((flipYawAngle * yawAngle) + 25);
        Logger.recordOutput("LineUp/ProcessedYawAngle", yawAngle);
        double xOffset = distance * Math.sin(yawAngle);
        Logger.recordOutput("LineUp/CameraXOffsetToAprilTag", xOffset);
        xOffset -= .082;
        if (Double.isInfinite(xOffset)) {
            return 0.0;
        } else {
            return (flipYawAngle * xOffset);
        }
    }

    public double robotYOffsetToAprilTag(Blackboard blackboard) {
        double yawAngle;
        double flipYawAngle;
        int cameraIndex = findClosestCamera(blackboard);
        if (blackboard.getTarget("target") != Targets.NONE) {
            if (blackboard.isTargetLeftBranch("target")) {
                yawAngle = inputs[3].latestTargetObservationDouble.tx();
                flipYawAngle = 1.0;
            } else {
                yawAngle = inputs[2].latestTargetObservationDouble.tx();
                flipYawAngle = -1.0;
            }
        } else {
            yawAngle = inputs[cameraIndex].latestTargetObservationDouble.tx();
            if (cameraIndex == 3) {
                flipYawAngle = 1.0;
            } else {
                flipYawAngle = -1.0;
            }
        }

        yawAngle = Units.degreesToRadians((-flipYawAngle * yawAngle) + 25);
        double distance = cameraDistanceToAprilTag(blackboard);

        double yOffset = distance * Math.cos(yawAngle);
        Logger.recordOutput("LineUp/CameraXOffset", yOffset);

        return yOffset + 0.185;
    }

    public double robotAngleToAprilTag(Blackboard blackboard) {
        double xOffset = robotXOffsetToAprilTag(blackboard);
        double yOffset = robotYOffsetToAprilTag(blackboard);

        double robotAngle = Math.atan2(xOffset, yOffset);
        return Units.radiansToDegrees(robotAngle);
    }

    public double robotDistanceToAprilTag(Blackboard blackboard) {
        double xOffset = robotXOffsetToAprilTag(blackboard);
        double yOffset = robotYOffsetToAprilTag(blackboard);

        double robotDistance = Math.hypot(xOffset, yOffset);
        return robotDistance;
    }

    // ONLY USE FOR LOGGING
    public void debugxOffset() {
        double YawAngle = inputs[2].latestTargetObservationDouble.tx();
        Logger.recordOutput("LineUp/2RawYawAngle", YawAngle);
        YawAngle = Units.degreesToRadians(YawAngle);
    }

    public boolean getLeftBranch(Blackboard blackboard) {
        if (blackboard.getTarget("target") != Targets.NONE) {
            return blackboard.isTargetLeftBranch("target");
        } else {
            return findClosestBranch(blackboard);
        }
       
    }

    public int findClosestCamera(Blackboard blackboard) {
        double leftCameraYaw = inputs[3].latestTargetObservationDouble.tx();
        double rightCameraYaw = inputs[2].latestTargetObservationDouble.tx();
        if (Math.abs(leftCameraYaw) < Math.abs(rightCameraYaw)) {
            return 3;
        } else {
            return 2;
        }
    }

    public boolean findClosestBranch(Blackboard blackboard) {
        if (findClosestCamera(blackboard) == 3) {
            //For left branch
            return true;
        } else {
            //For right branch
            return false;
        }
    }

    public Rotation2d getTargetRotation(Blackboard blackboard) {
        if (blackboard.get("target") != Targets.NONE) {
            return AllianceFlipUtil.apply(blackboard.getTargetRotation("target"));
        } else {
            return new Rotation2d();
        }
    }

    public int getBestTag() {
        return 0;
    }

    public Rotation2d getTagRotation(int AprilTagID) {
        Rotation2d targetRotation = new Rotation2d();
        switch(AprilTagID) {
            case 6:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchL.getRotation());
            break;
            case 7:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchA.getRotation());
            break;
            case 8:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchC.getRotation());
            break;
            case 9:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchE.getRotation());
            break;
            case 10:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchG.getRotation());
            break;
            case 11:
            targetRotation = AllianceFlipUtil.apply(Constants.reefBranchI.getRotation());
            break;
            case 17:
            targetRotation = Constants.reefBranchC.getRotation();
            break;
            case 18:
            targetRotation = Constants.reefBranchA.getRotation();
            break;
            case 19:
            targetRotation = Constants.reefBranchL.getRotation();
            break;
            case 20:
            targetRotation = Constants.reefBranchI.getRotation();
            break;
            case 21:
            targetRotation = Constants.reefBranchG.getRotation();
            break;
            case 22:
            targetRotation = Constants.reefBranchE.getRotation();
            break;
            default:
            targetRotation = new Rotation2d();
            break;
        }
        return targetRotation;
    }
    
}
