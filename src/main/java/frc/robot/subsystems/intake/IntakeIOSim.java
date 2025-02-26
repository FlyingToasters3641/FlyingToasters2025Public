package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.BehaviorTree.Blackboard;



public class IntakeIOSim implements IntakeIO {
    private MutVoltage IN_appliedVolts = Volts.mutable(0);
    private final IntakeSimulation intakeSimulation;
    private final SimulatedArena simulatedArena;
    private Blackboard blackboard;

    private final ProfiledPIDController IN_PID_Controller = new ProfiledPIDController(IntakeConstants.IN_PROFILED_PID_CONSTANTS.kP, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kI, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kD, IntakeConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final IntakeIO intakesimIO = new IntakeIO() {};
 

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
    
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation, SimulatedArena simulatedArena, Blackboard blackboard){
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                 driveTrainSimulation,
                Inches.of(19.0),
                Inches.of(19.0),
                IntakeSimulation.IntakeSide.LEFT,
                5);
        this.simulatedArena = simulatedArena;
        this.blackboard = blackboard;
    }
    
        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            IN_ARM_sim.update(.02);
            inputs.IN_angle.mut_replace(IN_ARM_sim.getAngleRads(), Radians);
            inputs.IN_voltage.mut_replace(IN_appliedVolts);
            inputs.IN_setpointAngle.mut_replace(IN_PID_Controller.getGoal().position, Degrees);      
            if(intakeSimulation.getGamePiecesAmount() >= 0){
                inputs.IN_hasAlgae = true;
            } else {
                inputs.IN_hasAlgae = false;
            }
            Logger.recordOutput("Intake/GamepieceCount", intakeSimulation.getGamePiecesAmount());
        }
     
        @Override
        public void IN_runSetpoint(Angle setpoint) {
            Angle currentAngle = Radians.of(IN_ARM_sim.getAngleRads());
            Logger.recordOutput("Intake/Setpoint", setpoint.in(Degrees));
            Voltage controllerVoltage = Volts.of(IN_PID_Controller.calculate(currentAngle.in(Degrees), setpoint.in(Degrees)));
            IN_runVolts(controllerVoltage);
        }
    
        @Override
        public void IN_runVolts(Voltage volts) {
            double appliedVoltage = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
            IN_appliedVolts.mut_replace(appliedVoltage, Volts);
            IN_ARM_sim.setInputVoltage(appliedVoltage);
        }

        @Override
        public void IN_setRunning(boolean runIntake) {
            Logger.recordOutput("Intake/Running", runIntake);
            if(runIntake){
                intakeSimulation.startIntake();
            } else {
                intakeSimulation.stopIntake();
            }
        }

        @Override
        public void IN_reverseIntake(boolean reverse, double speed) {
            if (reverse){
                intakesimIO.IN_setRoller(-speed);
                intakeSimulation.obtainGamePieceFromIntake();
            } else {
                intakesimIO.IN_setRoller(speed);
            }
        }
        
      
}