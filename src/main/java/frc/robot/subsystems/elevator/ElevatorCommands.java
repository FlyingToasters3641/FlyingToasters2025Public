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

    public static Command EL_goToL1 (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorConstants.EL_L1_HEIGHT));
    }

    public static Command EL_goToL2 (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorConstants.EL_L2_HEIGHT));
    }

    public static Command EL_goToL3 (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorConstants.EL_L3_HEIGHT));
    }

    public static Command EL_goToL4 (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorConstants.EL_L4_HEIGHT));
    }

    public static Command EL_goToNet (Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorConstants.EL_NET_HEIGHT));
    }

}
