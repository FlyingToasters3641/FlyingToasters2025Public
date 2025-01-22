package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    

    //TODO: Is this class really needed? Discuss with someone later about this -AU

    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    
    //Random value ;)
    public static final Distance METERS_PER_ROTATION = Meters.of(1); 

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

	public static final PIDConstants EL_PROFILED_PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);

    public static final FFConstants EL_FF_CONSTANTS = new FFConstants(0, 0, 0, 0);


}
