package frc.robot.subsystems.vision;

import java.util.List;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

import javax.print.attribute.standard.MediaSize.NA;

public class CameraVisualizer {

    private final String key;

    private final Transform3d robotToCamera;
    
    private Pose2d robotPose;

    private final Supplier<Pose2d> poseSupplier;

    //TODO: Consider moving this class to VisionIOPhotonVisionSim for clarity if needed.

    public CameraVisualizer(String key, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier){
        this.key = key;
        this.robotToCamera = robotToCamera;
        this.poseSupplier = poseSupplier;
    }
    
    public void update(Rotation2d robotRotation, List<PhotonTrackedTarget> targets) {

        robotPose = poseSupplier.get();
        double x = robotToCamera.getX();
        double y = robotToCamera.getY();
        double z = robotToCamera.getZ();

        double radius = Math.sqrt(Math.pow(y, 2)+Math.pow(x, 2));

        Pose3d cameraPose = new Pose3d(
            radius * Math.cos(robotRotation.getRadians() + Math.atan2(x/radius, y/radius)) + robotPose.getX(),
            radius * Math.sin(robotRotation.getRadians() + Math.atan2(x/radius, y/radius)) + robotPose.getY(),
            z,
            new Rotation3d(robotToCamera.getRotation().getX(), 
                robotToCamera.getRotation().getY(), 
                robotToCamera.getRotation().getZ()+robotRotation.getRadians()));
        
        
        int aprilTag0 = 0;
        if (targets.size() >= 1) {
            aprilTag0 = targets.get(0).getFiducialId();
        } else {
            aprilTag0 = 0;
        }
        Logger.recordOutput("Vision/Mechanism3d/" + key + "/Apriltags/", aprilTag0);
        Logger.recordOutput("Vision/Mechanism3d/" + key, cameraPose);
    }
}