package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {

    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    public static final Angle SERVO_RATCHET_ENGAGED = Degrees.of(0);
    public static final Angle SERVO_RATCHET_DISENGAGED = Degrees.of(90);
    public static final double SERVO_RATCHET_ENGAGED_DOUBLE = 0.0;
    public static final double SERVO_RATCHET_DISENGAGED_DOUBLE = 90.0;
    //Velocity threshold - climber has to be moving past this to keep climbing, Value is determined by the speed climbing
    public static final double STALL_VELOCITY_THRESHOLD = -19.0; // in encoder ticks per 100ms
    public static final double MIN_OUTPUT_PERCENT = 0.2;
    //Robot Extended Position to get ready to climb
    public static final double EXTENDED_POSITION = 260.0;
    //Robot Retracted Position for cycling and climbing
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
