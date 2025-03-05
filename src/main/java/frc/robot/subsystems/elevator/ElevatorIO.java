package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs
    {
        public MutDistance EL_position = Inches.mutable(0);
        public MutLinearVelocity EL_velocity = InchesPerSecond.mutable(0);

        public MutDistance position = Inches.mutable(0);

        public MutVoltage EL_voltage = Volts.mutable(0);

        public MutLinearVelocity setpointVelocity = InchesPerSecond.mutable(0);
        public MutDistance setpointPosition = Inches.mutable(0);
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setELPosition(double position) {};

    public default Distance EL_getPosition() {
            return Inches.of(0);}

    public default void ELStop() {};

    default void EL_runSetpoint(Distance position) {}

    default void EL_runVolts(Voltage volts) {}

    default void EL_setPID(double p, double i, double d) {}

    default void EL_setSpeed(double speed) {}

    
}
