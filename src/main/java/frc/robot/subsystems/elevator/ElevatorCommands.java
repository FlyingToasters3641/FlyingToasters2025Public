package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    

    //A few quick commands for general usage. Consider merging this file with Elevator.java later

    public static Command EL_setPosition (Elevator m_Elevator, Distance m_position){
        return Commands.run(() -> m_Elevator.EL_setpoint = m_position);
    }

    public static Command EL_stop (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.ELStop());
    }

}
