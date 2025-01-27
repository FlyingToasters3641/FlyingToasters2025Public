package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    private static final TalonFX IN_TalonFXOne = new TalonFX(3, CANbusName);// TODO: Update CANIDs
    private static final TalonFX IN_TalonFXTwo = new TalonFX(4, CANbusName); // TODO: Standardize Names for IntakeMotors

    private final ProfiledPIDController IN_PID_Controller = new ProfiledPIDController(IntakeConstants.IN_PROFILED_PID_CONSTANTS.kP, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kI, IntakeConstants.IN_PROFILED_PID_CONSTANTS.kD, IntakeConstants.TRAPEZOID_PROFILE_CONSTRAINTS);

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

    private final FlywheelSim IN_WHEEL_sim = new FlywheelSim(null, null, null);
        private double appliedVoltage;
    
    
        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            IN_ARM_sim.update(.02);
            
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


}