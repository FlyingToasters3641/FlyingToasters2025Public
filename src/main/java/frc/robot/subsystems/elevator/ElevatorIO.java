package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs
    {
        public MutDistance EL_position = Inches.mutable(0);
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setELPosition(Distance position) {};
    
}
