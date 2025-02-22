package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.lib.BehaviorTree.Blackboard;

public class VisionIOPhotonVision2D implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    private int targetedAprilTag = 0;

    public VisionIOPhotonVision2D(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        Set<Short> tagIds = new HashSet<>();

        for (var result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {

                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
                        result.getBestTarget().getSkew(),
                        result.getBestTarget().getArea());
                if (targetedAprilTag != 0) {
                    if (result.getBestTarget().getFiducialId() == targetedAprilTag) {
                        inputs.latestTargetObservationDouble = new TargetObservationDouble(
                                result.getBestTarget().getYaw(),
                                result.getBestTarget().getPitch());
                    }
                } else {
                    inputs.latestTargetObservationDouble = new TargetObservationDouble(
                            result.getBestTarget().getYaw(),
                            result.getBestTarget().getPitch());
                }
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(), 0.0, 0.0);
            }
        }

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    @Override
    public void getTargetID(Blackboard blackboard) {
        targetedAprilTag = blackboard.getTargetID("target");
    }

}
