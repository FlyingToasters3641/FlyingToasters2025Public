package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ScorerIO {
    @AutoLog
    public static class ScorerIOInputs {     
        
        public MutAngle CS_angle = Radians.mutable(0);
        public MutVoltage CS_voltage = Volts.mutable(0);
        public MutAngle CS_setpointAngle = Radians.mutable(0);
        public boolean CS_algae = false;
        public boolean CS_coral = false;
        
    }

    public default void updateInputs(ScorerIOInputs inputs) {};
   
    public default void setCSAngle(double angle) {};
    
    public default void CS_setRoller(double speed) {};

    public default void CSStop() {};

    public default void CS_runSetpoint(Angle angle) {}

    public default void CS_runVolts(Voltage volts) {}

    public default void CS_setPID(double p, double i, double d) {}

    public default void CS_reverseScorer(boolean reverse) {}

    public default void CS_shootSimAlgae(AbstractDriveTrainSimulation driveSimulation) {}

    public default void CS_shootL1SimCoral(AbstractDriveTrainSimulation driveTrainSimulation) {}

    public default void CS_shootL2SimCoral(AbstractDriveTrainSimulation driveTrainSimulation) {}

    public default void CS_shootL3SimCoral(AbstractDriveTrainSimulation driveTrainSimulation) {}

    public default void CS_shootL4SimCoral(AbstractDriveTrainSimulation driveTrainSimulation) {}


}
