package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {

    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    public static final Angle SERVO_RATCHET_ENGAGED = Degrees.of(0);
    public static final Angle SERVO_RATCHET_DISENGAGED = Degrees.of(180);
    public static final double STALL_VELOCITY_THRESHOLD = 10.0; // in encoder ticks per 100ms
    public static final double MIN_OUTPUT_PERCENT = 0.2;
    public static final double EXTENDED_POSITION = 0.0;
    public static final double RETRACTED_POSITION = 0.0;
    public static final int STALL_CURRENT_THRESHOLD = 20;

    public static class FFConstants{
        
        public double kS;
        public double kV;
        public double kA;
        public double kG;

        public FFConstants(double ks, double kg, double kv, double ka) {
            this.kS = ks;
            this.kV = kv;
            this.kA = ka;
            this.kG = kg;
        }

    }

	public static final PIDConstants CL_PROFILED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);

    public static final FFConstants CL_FF_CONSTANTS = new FFConstants(0, 0, 0, 0);
    
}
