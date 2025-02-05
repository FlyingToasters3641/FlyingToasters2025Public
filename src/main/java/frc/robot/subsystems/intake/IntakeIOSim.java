package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class IntakeIOSim implements IntakeIO {
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX IN_TalonFXOne = new TalonFX(3, CANbusName);// TODO: Update CANIDs
    private static final TalonFX IN_TalonFXTwo = new TalonFX(4, CANbusName); // TODO: Standardize Names for IntakeMotors
    private static final TalonFXSimState IN_TalonFXOneSim = new TalonFXSimState(IN_TalonFXOne);
	private static final TalonFXSimState IN_TalonFXTwoSim = new TalonFXSimState(IN_TalonFXTwo);;

        
    private MutVoltage IN_appliedVolts = Volts.mutable(0);
    

    private final ProfiledPIDController IN_PID_Controller = new ProfiledPIDController(IntakeConstants.IN_PROFILED_PID_CONSTANTS.kP, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kI, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kD, IntakeConstants.TRAPEZOID_PROFILE_CONSTRAINTS);

    private double appliedVoltage;
 

    private final SingleJointedArmSim IN_ARM_sim = new SingleJointedArmSim(
        IntakeConstants.kArmPlant,
        IntakeConstants.kKrakenX60Foc,
        IntakeConstants.kArmGearing,
        IntakeConstants.kArmLength,
        IntakeConstants.kArmMinAngle,
        IntakeConstants.kArmMaxAngle,
        IntakeConstants.kSimulateGravity,
        IntakeConstants.kArmStartAngle
    );

        
    
    public IntakeIOSim(){
        // this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        //         "Algae",
        //         driveTrainSimulation,
        //         Inches.of(19.0),
        //         Inches.of(19.0),
        //         IntakeSimulation.IntakeSide.BACK,
        //         1);
        // this.simulatedArena = simulatedArena;
       IN_TalonFXOneSim.setRawRotorPosition(IN_ARM_sim.getAngleRads() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));
		IN_TalonFXOneSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));

        IN_TalonFXTwoSim.setRawRotorPosition(IN_ARM_sim.getAngleRads() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));
        IN_TalonFXTwoSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));
    }
        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            IN_ARM_sim.update(.02);
         
        
            inputs.IN_angle.mut_replace(IN_ARM_sim.getAngleRads(), Radians);
            inputs.IN_voltage.mut_replace(appliedVoltage, Volts);
            inputs.IN_setpointAngle.mut_replace(IN_PID_Controller.getSetpoint().position, Radians);
        

            IN_TalonFXOneSim.setRawRotorPosition(IN_ARM_sim.getAngleRads() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));
		IN_TalonFXOneSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));

        IN_TalonFXTwoSim.setRawRotorPosition(IN_ARM_sim.getAngleRads() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));
        IN_TalonFXTwoSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec() / IntakeConstants.DEGREES_PER_ROTATION.in(Radians));

            // IN_TalonFXOneSim.setRawRotorPosition(IN_ARM_sim.getAngleRads());
            // IN_TalonFXTwoSim.setRawRotorPosition(IN_ARM_sim.getAngleRads());
            // IN_TalonFXOneSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec());
            // IN_TalonFXTwoSim.setRotorVelocity(IN_ARM_sim.getVelocityRadPerSec());
            
        }
    
        @Override
        public void IN_runSetpoint(Angle setpoint) {
            Voltage controllerVoltage = Volts.of(IN_PID_Controller.calculate(IN_ARM_sim.getAngleRads(), setpoint.in(Radians)));
            IN_runVolts(controllerVoltage);
        }
    
        @Override
        public void IN_runVolts(Voltage volts) {
            appliedVoltage = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
            IN_appliedVolts.mut_replace(appliedVoltage, Volts);
            IN_ARM_sim.setInputVoltage(appliedVoltage);
        }

        // @Override
        // public void IN_setRunning(boolean runIntake) {
        //     Logger.recordOutput("Intake/Running", runIntake);
        //     if(runIntake){
        //         intakeSimulation.startIntake();
        //     } else {
        //         intakeSimulation.stopIntake();
        //     }
        // }
        
        @Override
        public void IN_setPID(double p, double i, double d) {
            IN_PID_Controller.setPID(p, i, d);
        }

        @Override
        public void IN_reverseIntake(boolean reverse) {
            // if (reverse){
            //     intakeSimulation.obtainGamePieceFromIntake();

            // } else {
            // }
        }

      
       


}