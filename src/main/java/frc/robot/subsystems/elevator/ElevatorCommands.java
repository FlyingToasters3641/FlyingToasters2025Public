package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    

    
    public static Command EL_setPosition (Elevator m_Elevator, Distance m_position){
        return Commands.run(() -> m_Elevator.setpoint = m_position);
    }

    // public static Command EL_goToTestPosition(Elevator m_Elevator){
    //     return Commands.run(() -> ElevatorConstants.EL_setPosition(m_Elevator, 15));
    // }
}
