package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.BehaviorTree.Blackboard;

public class IntakeIOSim implements IntakeIO {
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX IN_TalonFXOne = new TalonFX(3, CANbusName);// TODO: Update CANIDs
    private static final TalonFX IN_TalonFXTwo = new TalonFX(4, CANbusName); // TODO: Standardize Names for IntakeMotors

    private final ProfiledPIDController IN_PID_Controller = new ProfiledPIDController(IntakeConstants.IN_PROFILED_PID_CONSTANTS.kP, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kI, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kD, IntakeConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final IntakeSimulation intakeSimulation;
    private double appliedVoltage;
    private final SimulatedArena simulatedArena;
    private Blackboard blackboard;

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
                IntakeSimulation.IntakeSide.BACK,
                1);
        this.simulatedArena = simulatedArena;
        this.blackboard = blackboard;
    }
        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            IN_ARM_sim.update(.02);
            Logger.recordOutput("Intake/GamepieceCount", intakeSimulation.getGamePiecesAmount());
            
        }
    
        @Override
        public void IN_runSetpoint(Angle setpoint) {
            IN_runVolts(IN_PID_Controller.calculate(IN_ARM_sim.getAngleRads(), setpoint.in(Radians)));
        }
    
        @Override
        public void IN_runVolts(double volts) {
            appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
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
        public void IN_reverseIntake(boolean reverse) {
            if (reverse){
                intakeSimulation.obtainGamePieceFromIntake();
            } else {
            }
        }


}