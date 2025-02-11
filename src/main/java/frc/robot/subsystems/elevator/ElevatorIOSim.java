package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ElevatorIOSim implements ElevatorIO{
    
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX EL_TalonFXOne = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    private static final TalonFX EL_TalonFXTwo = new TalonFX(2, CANbusName); // TODO: Standardize Names for ElevatorMotors
    private static final TalonFXSimState EL_TalonFXOneSim = new TalonFXSimState(EL_TalonFXOne);
	private static final TalonFXSimState EL_TalonFXTwoSim = new TalonFXSimState(EL_TalonFXTwo);;

    private final ProfiledPIDController EL_PID_controller = new ProfiledPIDController(ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kP, ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kI, ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kD, ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final ElevatorFeedforward EL_FeedForward = new ElevatorFeedforward(ElevatorConstants.EL_FF_CONSTANTS.kS,ElevatorConstants.EL_FF_CONSTANTS.kG , ElevatorConstants.EL_FF_CONSTANTS.kV);

    public MutDistance EL_simSetpoint = Inches.mutable(0);  
    
    private MutVoltage EL_appliedVolts = Volts.mutable(0);
    

    //new sim as of 1/30/2025
    private final ElevatorSim EL_sim = new ElevatorSim(
        ElevatorConstants.EL_kKrakenX60Foc,
        ElevatorConstants.EL_GEARING,
        ElevatorConstants.EL_CARRIAGE_MASS,
        ElevatorConstants.EL_DRUM_SPOOL_RADIUS,
        ElevatorConstants.EL_MIN_HEIGHT,
        ElevatorConstants.EL_MAX_HEIGHT,
        ElevatorConstants.EL_SIMULATE_GRAVITY,
        ElevatorConstants.EL_STARTING_HEIGHT
    );
    
    public ElevatorIOSim(){

        //Defines the TalonFX sims to be equal to the WPILib Sims

        EL_TalonFXOneSim.setRawRotorPosition(EL_sim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
		EL_TalonFXOneSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));

        EL_TalonFXTwoSim.setRawRotorPosition(EL_sim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
        EL_TalonFXTwoSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));

        
    }

    
    //Updates all the sim values and the logger values.
    @Override
    public void updateInputs(ElevatorIOInputs inputs){

        EL_sim.update(.02);

        inputs.EL_position.mut_replace(EL_sim.getPositionMeters(), Meters);
        inputs.EL_velocity.mut_replace(EL_sim.getVelocityMetersPerSecond(), InchesPerSecond);
;
        inputs.EL_voltage.mut_replace(EL_appliedVolts);

        inputs.position.mut_replace(EL_sim.getPositionMeters(), Meters);

        inputs.setpointPosition.mut_replace(EL_PID_controller.getSetpoint().position, Meters);
        inputs.setpointVelocity.mut_replace(0, MetersPerSecond);


        //TalonFX Sim Values
    
        EL_TalonFXOneSim.setRawRotorPosition(EL_sim.getPositionMeters());
        EL_TalonFXTwoSim.setRawRotorPosition(EL_sim.getPositionMeters());

        
        EL_TalonFXOneSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond());
        EL_TalonFXTwoSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond());

    }
        

    //Calculate to go to a specific point up into the elevator.
    @Override
    public void EL_runSetpoint(Distance position) {
        Distance currentHeight = Meters.of(EL_sim.getPositionMeters());

        Voltage controllerVoltage = Volts.of(EL_PID_controller.calculate(currentHeight.in(Inches), position.in(Inches)));


        EL_runVolts(controllerVoltage);
    }

    //Limits volts to go between a certain high and low value
    @Override
    public void EL_runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        EL_appliedVolts.mut_replace(clampedEffort, Volts);
        EL_sim.setInputVoltage(clampedEffort);
    }

    
    @Override
    public void EL_setPID(double p, double i, double d) {
        EL_PID_controller.setPID(p, i, d);
    }

    //Just in case
    @Override
    public void ELStop() {
        EL_runVolts(Volts.of(0));
    }







}
