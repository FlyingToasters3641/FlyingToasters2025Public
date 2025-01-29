package frc.robot.subsystems.scorer;

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

public class ScorerIOSim implements ScorerIO {
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX IN_TalonFXOne = new TalonFX(3, CANbusName);// TODO: Update CANIDs
    private static final TalonFX IN_TalonFXTwo = new TalonFX(4, CANbusName); // TODO: Standardize Names for ScorerMotors

    private final ProfiledPIDController CS_PID_Controller = new ProfiledPIDController(ScorerConstants.IN_PROFILED_PID_CONSTANTS.kP, ScorerConstants.IN_PROFILED_PID_CONSTANTS.kI, ScorerConstants.IN_PROFILED_PID_CONSTANTS.kD, ScorerConstants.TRAPEZOID_PROFILE_CONSTRAINTS);

    private double appliedVoltage;

    private final SingleJointedArmSim CS_ARM_sim = new SingleJointedArmSim(
        ScorerConstants.kArmPlant,
        ScorerConstants.kKrakenX60Foc,
        ScorerConstants.kArmGearing,
        ScorerConstants.kArmLength,
        ScorerConstants.kArmMinAngle,
        ScorerConstants.kArmMaxAngle,
        ScorerConstants.kSimulateGravity,
        ScorerConstants.kArmStartAngle
    );

        @Override
        public void updateInputs(ScorerIOInputs inputs) {
            CS_ARM_sim.update(.02);
            
        }
    
        @Override
        public void CS_runSetpoint(Angle setpoint) {
            CS_runVolts(CS_PID_Controller.calculate(CS_ARM_sim.getAngleRads(), setpoint.in(Radians)));
        }
    
        @Override
        public void CS_runVolts(double volts) {
            appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
            CS_ARM_sim.setInputVoltage(appliedVoltage);
        }

        @Override
        public void CS_setRunning(boolean runScorer) {
            Logger.recordOutput("Scorer/Running", runScorer);
        }

        @Override
        public void CS_reverseScorer(boolean reverse) {
        }


}