package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.Angle;

public class ClimberCommands {

    public static Command CL_setRotation (Climber m_Climber, Angle m_rotation){
        return Commands.run(() -> m_Climber.CL_setpoint = m_rotation);
    }

    public static Command CL_stop (Climber m_Climber){
        return Commands.run(() -> m_Climber.CLStop());
    }

    public static Angle CL_getRotation (Climber m_Climber){
        return m_Climber.getCLRotation();
    }
}
