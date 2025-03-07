package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {     
        
        public MutAngle IN_angle = Radians.mutable(0);
        public MutVoltage IN_voltage = Volts.mutable(0);
        public MutAngle IN_setpointAngle = Radians.mutable(0);

        public boolean IN_hasAlgae = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};
   
    public default void setINAngle(double angle) {};
    
    public default void IN_setRunning(boolean runIntake) {};

    public default void INStop() {};

    public default void IN_runSetpoint(Angle angle) {}
   
    public default void IN_setRoller(double speed) {}

    public default void IN_runVolts(Voltage volts) {}

    public default void IN_setPID(double p, double i, double d) {}

    public default void IN_reverseIntake(boolean reverse, double speed) {}

    public default void IN_simIntakeAlgae(boolean runIntake) {}   
  
    

}
