package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ScorerIOSim implements ScorerIO {
    private final ProfiledPIDController CS_PID_Controller = new ProfiledPIDController(ScorerConstants.CS_PROFILED_PID_CONSTANTS.kP, ScorerConstants.CS_PROFILED_PID_CONSTANTS.kI, ScorerConstants.CS_PROFILED_PID_CONSTANTS.kD, ScorerConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final ArmFeedforward CS_FeedForward = new ArmFeedforward(0, 0, 0, 0);
    private MutVoltage CS_appliedVoltage = Volts.mutable(0);
    private final SimDevice CS_algaeDevice = SimDevice.create("AlgaeSwitchDevice");
    private final SimBoolean CS_algaeSwitch = CS_algaeDevice.createBoolean("AlgaeSwitch", Direction.kInput, false);
    private final SimDevice CS_coralDevice = SimDevice.create("CoralSwitchDevice");
    private final SimBoolean CS_coralSwitch = CS_coralDevice.createBoolean("CoralSwitch", Direction.kInput, false);

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
            inputs.CS_angle.mut_replace(CS_ARM_sim.getAngleRads(), Radians);
            inputs.CS_voltage.mut_replace(CS_appliedVoltage);
            inputs.CS_setpointAngle.mut_replace(this.CS_PID_Controller.getGoal().position, Degrees);
            inputs.CS_algae = CS_algaeSwitch.get();
            inputs.CS_coral = CS_coralSwitch.get();
        }
    
        @Override
        public void CS_runSetpoint(Angle setpoint) {
            Angle currentAngle = Radians.of(CS_ARM_sim.getAngleRads());
            Logger.recordOutput("Scorer/Setpoint", setpoint.in(Degrees));
            Angle setpointAngle = Degrees.of(CS_PID_Controller.getSetpoint().position);
            AngularVelocity setpointVelocity = DegreesPerSecond.of(CS_PID_Controller.getSetpoint().velocity);
            Logger.recordOutput("Scorer/CurrentAngle", currentAngle.in(Degrees));
            Voltage controllerVoltage = Volts.of(CS_PID_Controller.calculate(currentAngle.in(Degrees), setpoint.in(Degrees)));

            //Voltage feedForwardVoltage = Volts.of(CS_FeedForward.calculate(setpointAngle.in(Radians), setpointVelocity.in(RadiansPerSecond)));            

            CS_runVolts(controllerVoltage);
        }
    
        @Override
        public void CS_runVolts(Voltage volts) {
            double appliedVoltage = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
            CS_appliedVoltage.mut_replace(appliedVoltage, Volts);
            CS_ARM_sim.setInputVoltage(appliedVoltage);
        }

        @Override
        public void CS_setRoller(double speed) {
            Logger.recordOutput("Scorer/Running", speed);
        }

        @Override
        public void CS_reverseScorer(boolean reverse) {
        }

        @Override
        public void CS_setPID(double p, double i, double d) {
            CS_PID_Controller.setPID(p, i, d);
        }



}