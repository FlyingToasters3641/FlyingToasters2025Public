package frc.robot.subsystems.scorer;

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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ScorerIOSim implements ScorerIO {
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX CS_TalonFXOne = new TalonFX(3, CANbusName);// TODO: Update CANIDs
    private static final TalonFXSimState CS_TalonFXOneSim = new TalonFXSimState(CS_TalonFXOne);

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

    public ScorerIOSim(){
        CS_TalonFXOneSim.setRawRotorPosition(CS_ARM_sim.getAngleRads() / ScorerConstants.DEGREES_PER_ROTATION.in(Radians));
        CS_TalonFXOneSim.setRotorVelocity(CS_ARM_sim.getVelocityRadPerSec() / ScorerConstants.DEGREES_PER_ROTATION.in(Radians));

    }

        @Override
        public void updateInputs(ScorerIOInputs inputs) {
            CS_ARM_sim.update(.02);

            inputs.CS_angle.mut_replace(CS_ARM_sim.getAngleRads(), Radians);
            inputs.CS_voltage.mut_replace(appliedVoltage, Volts);
            inputs.CS_setpointAngle.mut_replace(CS_PID_Controller.getSetpoint().position, Radians);
            

            CS_TalonFXOneSim.setRawRotorPosition(CS_ARM_sim.getAngleRads() / ScorerConstants.DEGREES_PER_ROTATION.in(Radians));
            CS_TalonFXOneSim.setRotorVelocity(CS_ARM_sim.getVelocityRadPerSec() / ScorerConstants.DEGREES_PER_ROTATION.in(Radians));
        }
    
        @Override
        public void CS_runSetpoint(Angle setpoint) {
            Voltage controllerVoltage = Volts.of(CS_PID_Controller.calculate(CS_ARM_sim.getAngleRads(), setpoint.in(Radians)));
            CS_runVolts(controllerVoltage);
        }
    
        @Override
        public void CS_runVolts(Voltage volts) {
            appliedVoltage = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
            CS_ARM_sim.setInputVoltage(appliedVoltage);
        }

        @Override
        public void CS_setRoller(double speed) {
            Logger.recordOutput("Scorer/Running", speed);
        }

        @Override
        public void CS_reverseScorer(boolean reverse) {
        }


}