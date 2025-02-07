package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class PathFindToPath extends CommandOnFly {
    public PathFindToPath(
        Drive driveSubsystem,
        Supplier<PathPlannerPath> path) {
        super(() -> AutoBuilder.pathfindThenFollowPath(
                    path.get(), Constants.constraints)
                    );
    }

    
}
