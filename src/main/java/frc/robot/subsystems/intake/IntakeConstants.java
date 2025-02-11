package frc.robot.subsystems.intake;

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
import edu.wpi.first.units.measure.MomentOfInertia;

public class IntakeConstants {
    public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);;
    public static final PIDConstants IN_PROFILED_PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);
    public static double IDLE = 0.0;
    
    public static final DCMotor kKrakenX60Foc = DCMotor.getKrakenX60Foc(2);
    public static final double INTAKE_ANGLE_MODIFIER = Degrees.of(0).in(Radians);
    public static double kArmStartAngle = Units.Degrees.of(17).in(Units.Radians);
    public static double kArmGearing = 26.4;
    public static double kArmMinAngle = Units.Degrees.of(17).in(Units.Radians);
    public static double kArmMaxAngle = Units.Degrees.of(-48).in(Units.Radians);
    public static double kArmLength = Units.Inches.of(21.6).in(Units.Meters);
    public static MomentOfInertia kArmMomentOfInertia = Units.KilogramSquareMeters.of(0.3666671254);
    public static LinearSystem<N2, N1, N2> kArmPlant = LinearSystemId.createSingleJointedArmSystem(kKrakenX60Foc, kArmMomentOfInertia.baseUnitMagnitude(), kArmGearing);
    public static boolean kSimulateGravity = true;
}

