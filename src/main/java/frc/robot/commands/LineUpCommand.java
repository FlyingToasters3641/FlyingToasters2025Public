package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class LineUpCommand extends Command {
    private final Drive drive;
    private PathPlannerPath path;
    PathConstraints constraints;

    public LineUpCommand(Drive m_drive) {
        drive = m_drive;
        addRequirements(m_drive);

    }

    @Override
    public void initialize(){
        
        constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        try{
            path = PathPlannerPath.fromPathFile("lineUp");
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
    }

    @Override
    public void execute(){

        AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);

    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}


}
