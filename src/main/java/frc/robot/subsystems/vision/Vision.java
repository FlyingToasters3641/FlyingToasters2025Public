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
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

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
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
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

                

            Logger.recordOutput("LineUp/RobotXOffset", robotXOffsetToAprilTag());
            Logger.recordOutput("LineUp/CameraDistanceToAprilTag", cameraDistanceToAprilTag());
            Logger.recordOutput("LineUp/RobotYOffset", robotYOffsetToAprilTag());
            Logger.recordOutput("LineUp/RobotAngle", robotAngleToAprilTag());
            Logger.recordOutput("LineUp/RobotDistanceToAprilTag", robotDistanceToAprilTag());
            CenterXDistance = robotXOffsetToAprilTag();
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    // public double CenterAngleToAprilTag() {
    //     double PitchAngle = inputs[3].latestTargetObservationDouble.ty();
    //     double YawAngle = inputs[3].latestTargetObservationDouble.tx();



    //     double xDist = 0.0624 * Math.cos(YawAngle) / Math.tan(PitchAngle);
    //     return Math.atan((xDist * Math.tan(YawAngle) - 0.082) / (xDist + 0.185));
    // }


    public double cameraDistanceToAprilTag() {
        double pitchAngle = inputs[3].latestTargetObservationDouble.ty();
        pitchAngle = Units.degreesToRadians(pitchAngle);

        double distance = .0624 / Math.tan(pitchAngle);
        return distance;
    }

    public double robotXOffsetToAprilTag() {
        double distance = cameraDistanceToAprilTag();
        double yawAngle = inputs[3].latestTargetObservationDouble.tx();
        Logger.recordOutput("LineUp/RawYawAngle", yawAngle);

        yawAngle = Units.degreesToRadians(yawAngle + 25);
        Logger.recordOutput("LineUp/ProcessedYawAngle", yawAngle);

        double xOffset = distance * Math.sin(yawAngle);
        Logger.recordOutput("LineUp/CameraXOffset", xOffset);

        if (Double.isInfinite(xOffset)) {
            return 0.0;
        } else {
        return xOffset - 0.082;
        }
    }

    public double robotYOffsetToAprilTag() {
        double yawAngle = inputs[3].latestTargetObservationDouble.tx();
        yawAngle = Units.degreesToRadians(yawAngle + 25);
        double distance = cameraDistanceToAprilTag();

        double yOffset = distance * Math.cos(yawAngle);
        Logger.recordOutput("LineUp/CameraXOffset", yOffset);

        return yOffset + 0.185;
    }

    public double robotAngleToAprilTag() {
        double xOffset = robotXOffsetToAprilTag();
        double yOffset = robotYOffsetToAprilTag();

        double robotAngle = Math.atan2(xOffset, yOffset);
        return Units.radiansToDegrees(robotAngle);
    }

    public double robotDistanceToAprilTag() {
        double xOffset = robotXOffsetToAprilTag();
        double yOffset = robotYOffsetToAprilTag();

        double robotDistance = Math.hypot(xOffset, yOffset);
        return robotDistance;
    }

    public Rotation2d robotAngleToAprilTagRotation2d() {
        return Rotation2d.fromDegrees(robotAngleToAprilTag());
    }
}
