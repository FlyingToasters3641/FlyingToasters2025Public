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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;
    
    private final Supplier<Rotation2d> rotationSupplier;
    
    private CameraVisualizer cameraVisualizer;
    
    private int aprilTagID = 0;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier,  Supplier<Rotation2d> rotationSupplier, boolean wireframe) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;
        this.rotationSupplier = rotationSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);

        
        cameraVisualizer = new CameraVisualizer(name, robotToCamera, poseSupplier);
        cameraSim.enableDrawWireframe(wireframe);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        var debugField = visionSim.getDebugField();
        debugField.getObject("EstimatedRobot").setPose(poseSupplier.get());

        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        if (targets.size() != 0) {
            aprilTagID = targets.get(0).getFiducialId();
        } else {
            aprilTagID = 0;
        }

        super.updateInputs(inputs);
        
        cameraVisualizer.update(rotationSupplier.get(), targets); 
    }
}
