package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    


    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    
  
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


    public static final DCMotor EL_kKrakenX60Foc = DCMotor.getKrakenX60Foc(2);
    public static final double EL_GEARING = 5;
    public static final double EL_CARRIAGE_MASS = Pounds.of(10).in(Kilograms);
    public static final double EL_DRUM_SPOOL_RADIUS = Inches.of(1).in(Meters);
    public static final double EL_MIN_HEIGHT = Inches.zero().in(Meters);
    public static final double EL_MAX_HEIGHT = Inches.of(52.5).in(Meters);
    public static final boolean EL_SIMULATE_GRAVITY = true;
    public static final double EL_STARTING_HEIGHT = Inches.of(10.25).in(Meters);

	public static final PIDConstants EL_PROFILED_PID_CONSTANTS = new PIDConstants(0.625, 0, 0, 0.05);

    public static final FFConstants EL_FF_CONSTANTS = new FFConstants(0, 0, 0, 0);

    public static class EL_talonConfig{
        public static final double SENSOR_TO_MECHANISM_RATIO = (1/(3.10752688*Math.PI));
        public static final double ROTOR_TO_SENSOR_RATIO = 5.0;
        public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};
        public static final double MM_CRUISE_VELOCITY = 200;
        public static final double MM_ACCELERATION = 1000;
        public static final double PIVOT_MAGNET_OFFSET = 0.108154;
        public static ProfileConstraints MM_PROFILE_CONSTRAINTS = new ProfileConstraints(MM_CRUISE_VELOCITY, MM_ACCELERATION);
        public static double kP = 40.0;
        public static double kD = 1.0;
        public static double AkP = 40.0;
        public static double AkD = 3.0;

    }

    //ELEVATOR HEIGHTS
    public static final double EL_REST_HEIGHT = 0.2;
    public static final double EL_L1_HEIGHT = 0.2;
    public static final double EL_L2_HEIGHT = 7;
    public static final double EL_L3_HEIGHT = 22;
    public static final double EL_L4_HEIGHT = 52;
    public static final double EL_NET_HEIGHT = 52;
    public static final double EL_MIDPOINT_HEIGHT = 25;
    


}
