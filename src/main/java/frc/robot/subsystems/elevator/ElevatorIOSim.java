package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
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

    
    

    private final ElevatorSim EL_sim = new ElevatorSim(
        DCMotor.getKrakenX60(4),
        4, 
        Pounds.of(9.8).in(Kilograms), //TODO: Ask for Intake Mass 
        Inches.of(2).in(Meters), //TODO: Ask for Spool Radius
        Inches.of(22.4).in(Meters), 
        Inches.of(77).in(Meters), 
        true, 
        Inches.of(0).in(Meters)
    );
    
    public ElevatorIOSim(){
        EL_TalonFXTwo.setControl(new Follower(EL_TalonFXOne.getDeviceID(), true)); //This probably doesn't work LOL, be prepared to change if needed.

        EL_TalonFXOneSim.setRawRotorPosition(EL_sim.getPositionMeters() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
		EL_TalonFXOneSim.setRotorVelocity(EL_sim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_ROTATION.in(Meters));
    }

    

    @Override
    public void updateInputs(ElevatorIOInputs inputs){

        EL_sim.update(.02);

        inputs.EL_position.mut_replace(EL_sim.getPositionMeters(), Meters);
    }


    //TODO: Figure out how to convert voltage into movement of position. TRY PHOENIX SIM!
    @Override
    public void setELPosition(Distance position){
        Distance currentHeight = Meters.of(EL_sim.getPositionMeters());
        LinearVelocity currentVelocity = MetersPerSecond.of(EL_sim.getVelocityMetersPerSecond());



        
    }

}
