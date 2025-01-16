package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class PathFindToPose extends CommandOnFly {
    public PathFindToPose(
            Drive driveSubsystem,
            Supplier<Pose2d> targetPose,
            double speedMultiplier,
            double goalEndVelocity) {
        super(() -> AutoBuilder.pathfindToPose(
                        targetPose.get(), Constants.constraints, goalEndVelocity)
                );
    }
}
