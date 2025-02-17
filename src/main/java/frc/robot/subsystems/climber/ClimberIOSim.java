package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {

    public static final double CL_RATIO = 0.0;
    public static final double CL_ENCODER_RATIO = 0.0;

    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    public static final TalonFX CL_TalonFX = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    public static final CANcoder CL_CANCoder = new CANcoder(5, CANbusName); // TODO: Clarify CANcoder usage

    private static final TalonFXSimState CL_TalonFXSim = new TalonFXSimState(CL_TalonFX);

    private final ProfiledPIDController CL_PID_controller = new ProfiledPIDController(ClimberConstants.CL_PROFILED_PID_CONSTANTS.kP, ClimberConstants.CL_PROFILED_PID_CONSTANTS.kI, ClimberConstants.CL_PROFILED_PID_CONSTANTS.kD, ClimberConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final ElevatorFeedforward CL_FeedForward = new ElevatorFeedforward(ClimberConstants.CL_FF_CONSTANTS.kS,ClimberConstants.CL_FF_CONSTANTS.kG , ClimberConstants.CL_FF_CONSTANTS.kV);


    public MutAngle CL_simSetpoint = Radians.mutable(0);  
    
    private MutVoltage CL_appliedVolts = Volts.mutable(0);

    MutAngle setpoint = Radians.mutable(0);

    private final SingleJointedArmSim CL_sim = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(2),
        5,
        Kilograms.of(10).in(Pounds),
        Inches.of(.07225).in(Meters),
        Inches.of(0).in(Meters),
        Inches.of(26.25).in(Meters),
        true,
        Inches.of(10.25).in(Meters)
    );

    public ClimberIOSim() {
        CL_TalonFXSim.setRawRotorPosition(CL_sim.getAngleRads());
		CL_TalonFXSim.setRotorVelocity(CL_sim.getVelocityRadPerSec());
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){

        CL_sim.update(.02);

        inputs.CL_rotation.mut_replace(CL_sim.getAngleRads(), Radians);
        inputs.CL_angularVelocity.mut_replace(CL_sim.getVelocityRadPerSec(), RadiansPerSecond);
;
        inputs.CL_voltage.mut_replace(CL_appliedVolts);

        inputs.rotation.mut_replace(CL_sim.getAngleRads(), Radians);

        inputs.setpointRotation.mut_replace(CL_PID_controller.getSetpoint().position, Radians);
        inputs.setpointAngularVelocity.mut_replace(1, RadiansPerSecond);

        //TalonFX Sim Values
    
        CL_TalonFXSim.setRawRotorPosition(CL_sim.getAngleRads());

        
        CL_TalonFXSim.setRotorVelocity(CL_sim.getVelocityRadPerSec());
    }

    //Calculate to go to a specific point up into the elevator.
    @Override
    public void CL_runSetpoint(Angle rotation) {
        double currentAngle = CL_sim.getAngleRads();
        double currentVelocity = CL_sim.getVelocityRadPerSec();

        Voltage controllerVoltage = Volts.of(CL_PID_controller.calculate(currentAngle, new TrapezoidProfile.State(currentAngle, currentVelocity)));
        Voltage feedForwardVoltage = Volts.of(CL_FeedForward.calculate(currentVelocity));

        Voltage effort = controllerVoltage.plus(feedForwardVoltage);

        CL_runVolts(effort);
    }

    //Limits volts to go between a certain high and low value
    @Override
    public void CL_runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        CL_appliedVolts.mut_replace(clampedEffort, Volts);
        CL_sim.setInputVoltage(clampedEffort);
    }

    //Just in case
    @Override
    public void CL_Stop() {
        CL_runVolts(Volts.of(0));
    }

    @Override
    public Angle CL_getRotation() {
        return Radians.of(CL_sim.getAngleRads());
    }
}
