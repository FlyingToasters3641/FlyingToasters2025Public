package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    

    //Moves the elevator to a test position
    public static Command moveToTestPos(Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.setELPosition(22)); //22 is just some random number that i came up with on the fly.
    }

    public static Command moveToTestRestPos(Elevator m_Elevator){
        return Commands.run(() -> m_Elevator.setELPosition(0)); 
    }
}
