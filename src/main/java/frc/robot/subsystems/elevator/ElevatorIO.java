package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs
    {
        double EL_position = 0.0;
        double EL_position_inches = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setELPosition(double position) {};
    
}
