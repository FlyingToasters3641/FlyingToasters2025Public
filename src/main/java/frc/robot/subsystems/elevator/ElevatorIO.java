package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs
    {
        public MutDistance EL_position = Inches.mutable(0);
        public MutLinearVelocity EL_velocity = InchesPerSecond.mutable(0);

        public MutDistance EL_Goalpoint = Inches.mutable(0);
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setELPosition(double position) {};

    public default void ELStop() {};

    public default void setELGoal(double position) {};

    public default void applyELVolts() {};


    
}
