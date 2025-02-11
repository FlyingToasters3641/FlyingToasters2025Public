package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {
    
        public MutAngle CL_rotation = Radians.mutable(0);
        public MutAngularVelocity CL_angularVelocity = RadiansPerSecond.mutable(0);

        public MutAngle rotation = Radians.mutable(0);

        public MutVoltage CL_voltage = Volts.mutable(0);

        public MutAngle setpointRotation = Radians.mutable(0);
        public MutAngularVelocity setpointAngularVelocity = RadiansPerSecond.mutable(0);
    }

    public default void updateInputs(ClimberIOInputs inputs) {};

    public default void CL_setRotation(double rotation) {};

    public default void CL_runSetpoint(Angle rotation) {};

    public default void CL_runVolts(Voltage volts) {};

    public default void CL_setPID(double p, double i, double d) {};

    public default void CL_Stop() {};

    public default Angle CL_getRotation() {return Radians.of(0);};

    public default void CL_setServo(double angle) {};

    public default void CL_Home() {};

    public default void CL_setPosition(double position) {};

}
