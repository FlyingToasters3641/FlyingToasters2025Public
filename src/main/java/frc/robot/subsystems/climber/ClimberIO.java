package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {
    
        
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

}
