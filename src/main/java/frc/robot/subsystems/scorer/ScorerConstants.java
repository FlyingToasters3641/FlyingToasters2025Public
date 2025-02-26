package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ScorerConstants {

    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 720);;
    public static final PIDConstants CS_PROFILED_PID_CONSTANTS = new PIDConstants(0.01, 0, 0.01, 0);
    public static double IDLE = 0.0;
    public static final DCMotor kKrakenX60Foc = DCMotor.getKrakenX60Foc(2);
    public static final double SCORER_ANGLE_MODIFIER = Degrees.of(0).in(Radians);
    public static final Angle DEGREES_PER_ROTATION = Degrees.of(360); //FIXME: Update with correct value
    public static final double SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double ROTOR_TO_SENSOR_RATIO = 17.14;
    public static final double CS_L4_ANGLE = 200;
    public static double kArmStartAngle = Units.Degrees.of(0).in(Units.Radians);
    public static double kArmGearing = 17.14;
    public static double kArmMinAngle = Units.Degrees.of(0).in(Units.Radians);
    public static double kArmMaxAngle = Units.Degrees.of(270).in(Units.Radians);
    public static double kArmLength = Units.Inches.of(21.6).in(Units.Meters);
    public static MomentOfInertia kArmMomentOfInertia = Units.KilogramSquareMeters.of(0.1554335034);
    public static LinearSystem<N2, N1, N2> kArmPlant = LinearSystemId.createSingleJointedArmSystem(kKrakenX60Foc, kArmMomentOfInertia.baseUnitMagnitude(), kArmGearing);
    public static boolean kSimulateGravity = true;
    public static Angle CS_SCORE_NET = Units.Degrees.of(160.0);


    public static class CS_pivotConfig {
        public static final double SENSOR_TO_MECHANISM_RATIO = 1;
        public static final double ROTOR_TO_SENSOR_RATIO = 26.4;
        public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};
        public static final double MM_CRUISE_VELOCITY = 2 * Math.PI;
        public static final double MM_ACCELERATION = (1 * Math.PI);
        public static final double PIVOT_MAGNET_OFFSET = 0.108154;
        public static ProfileConstraints MM_PROFILE_CONSTRAINTS = new ProfileConstraints(MM_CRUISE_VELOCITY, MM_ACCELERATION);
        public static double kG = 0.19;
        public static double kP = 2800.0;
        public static double kI = 0;
        public static double kD = 200.0;
        public static double kV = 3.28;

    }
    
}
