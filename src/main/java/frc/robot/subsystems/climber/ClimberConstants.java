package frc.robot.subsystems.climber;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ClimberConstants {

    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);


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
