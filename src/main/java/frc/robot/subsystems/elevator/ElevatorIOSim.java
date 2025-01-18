package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO{
    
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX EL_TalonFXOne = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    private static final TalonFX EL_TalonFXTwo = new TalonFX(2, CANbusName); // TODO: Standardize Names for Motors
    private static final TalonFXSimState EL_TalonFXOneSim = new TalonFXSimState(EL_TalonFXOne);
	private static final TalonFXSimState EL_TalonFXTwoSim = new TalonFXSimState(EL_TalonFXTwo);;

    private final ProfiledPIDController EL_PID_controller = new ProfiledPIDController(ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kP, ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kI, ElevatorConstants.EL_PROFILED_PID_CONSTANTS.kD, ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final ElevatorFeedforward EL_FeedForward = new ElevatorFeedforward(ElevatorConstants.EL_FF_CONSTANTS.kS,ElevatorConstants.EL_FF_CONSTANTS.kG , ElevatorConstants.EL_FF_CONSTANTS.kV);

    public MutDistance simSetpoint = Inches.mutable(0);  
    
    
    public MutDistance simGoal = Inches.mutable(0);

    private double appliedVolts = 0;
    
    
    //old sim
    // private final ElevatorSim EL_sim = new ElevatorSim(
    //     DCMotor.getKrakenX60(2),
    //     1, 
    //     Pounds.of(9.8).in(Kilograms), //TODO: Ask for Intake Mass 
    //     Inches.of(2).in(Meters), //TODO: Ask for Spool Radius
    //     Inches.of(22.4).in(Meters), 
    //     Inches.of(77).in(Meters), 
    //     true, 
    //     Inches.of(0).in(Meters)
    // );

    //new sim as of 1/18/2025
    private final ElevatorSim EL_sim = new ElevatorSim(
        DCMotor.getKrakenX60Foc(2),
        5,
        Kilograms.of(7.5).in(Kilograms),
        Inches.of(.07225).in(Meters),
        Inches.of(0).in(Meters),
        Inches.of(26.25).in(Meters),
        true,
        Inches.of(10.25).in(Meters)
    );
    
    public ElevatorIOSim(){



        EL_TalonFXOneSim.setRawRotorPosition(EL_sim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
		EL_TalonFXOneSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));

        EL_TalonFXTwoSim.setRawRotorPosition(EL_sim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
        EL_TalonFXTwoSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));

        
    }

    

    @Override
    public void updateInputs(ElevatorIOInputs inputs){

        EL_sim.update(.02);

        inputs.EL_position.mut_replace(EL_sim.getPositionMeters(), Meters);
        inputs.EL_velocity.mut_replace(EL_sim.getVelocityMetersPerSecond(), InchesPerSecond);

        inputs.EL_Goalpoint.mut_replace(simGoal.copy());

        EL_TalonFXOneSim.setRawRotorPosition(EL_sim.getPositionMeters());
        EL_TalonFXTwoSim.setRawRotorPosition(EL_sim.getPositionMeters());

        
        EL_TalonFXOneSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond());
        EL_TalonFXTwoSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond());
        

    }


    //TODO: Figure out how to convert voltage into movement of position. TRY PHOENIX SIM!
    // @Override
    // public void setELPosition(double position){
       
    //     simSetpoint.mut_replace(position, Inches);

    //     EL_sim.setState(position, 0.1); //another random value, lol, lmao even.

    //     EL_TalonFXOneSim.setRawRotorPosition(position);
    //     EL_TalonFXTwoSim.setRawRotorPosition(position);
        
    // }

    // @Override
    // public void ELStop(){
    //     this.setELPosition(0); //TODO: Update this to be consistent with setpoint stuff
    // }

    //trying out voltage sim
    @Override
    public void setELGoal(double position){
        simGoal.mut_replace(position, Inches);
        EL_PID_controller.setGoal(position);

        //Voltage effort = EL_PID_controller.calculate(EL_sim.getPositionMeters()).plus(EL_FeedForward.calculate(EL_PID_controller.getSetpoint().velocity.getValueAsDouble()));

        // appliedVolts.mut_replace(EL_PID_controller.calculate(EL_sim.getPositionMeters()) + EL_FeedForward.calculate(EL_PID_controller.getSetpoint().velocity),Volts);
		// EL_sim.setInputVoltage(appliedVolts.in(Volts));

        appliedVolts = EL_PID_controller.calculate(EL_sim.getPositionMeters()) + EL_FeedForward.calculate(EL_PID_controller.getSetpoint().velocity);

        EL_sim.setInputVoltage(appliedVolts);
    }



}
