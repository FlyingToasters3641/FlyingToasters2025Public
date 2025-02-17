package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

public class ScorerIOTalonFX implements ScorerIO{
    DigitalInput coralSensor = new DigitalInput(0);
    DigitalInput algaeSensor = new DigitalInput(1);
    private static final String CANbusName = "scorer";
    public static final TalonFX CS_pivotTalonFX = new TalonFX(3, CANbusName);
    public static final TalonFX CS_rollerTalonFX = new TalonFX(4, CANbusName);
    public static final CANcoder CS_pivotCANcoder = new CANcoder(5, CANbusName);
    public static final DigitalInput CS_coralSensor = new DigitalInput(0);
    public static final DigitalInput CS_algaeSensor = new DigitalInput(1);

    public ScorerIOTalonFX(){

        /*
         * CS_pivotConfig is a TalonFXConfiguration object that holds all of the settings for the pivot motor.
         */
        TalonFXConfiguration CS_pivotConfig = new TalonFXConfiguration();

        CS_pivotConfig.Feedback.FeedbackRemoteSensorID = CS_pivotCANcoder.getDeviceID();
        CS_pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        CS_pivotConfig.Feedback.SensorToMechanismRatio = ScorerConstants.CS_pivotConfig.SENSOR_TO_MECHANISM_RATIO;
        CS_pivotConfig.Feedback.RotorToSensorRatio = ScorerConstants.CS_pivotConfig.ROTOR_TO_SENSOR_RATIO;

        CS_pivotConfig.Slot0.kG = ScorerConstants.CS_pivotConfig.kG;
        CS_pivotConfig.Slot0.kP = ScorerConstants.CS_pivotConfig.kP;
        CS_pivotConfig.Slot0.kI = ScorerConstants.CS_pivotConfig.kI;
        CS_pivotConfig.Slot0.kD = ScorerConstants.CS_pivotConfig.kD;

        CS_pivotConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicCruiseVelocity(
            Units.radiansToRotations(ScorerConstants.CS_pivotConfig.MM_CRUISE_VELOCITY))
            .withMotionMagicAcceleration(ScorerConstants.CS_pivotConfig.MM_ACCELERATION);

        CS_pivotTalonFX.getConfigurator().apply(CS_pivotConfig);

        /*
         * CS_pivotCANcoderConfig is a TalonFXConfiguration object that holds all of the settings for the CANcoder on the pivot motor.
         */
        CANcoderConfiguration CS_pivotCANcoderConfig = new CANcoderConfiguration();
        CS_pivotCANcoderConfig.MagnetSensor.MagnetOffset = ScorerConstants.CS_pivotConfig.PIVOT_MAGNET_OFFSET;
        
        CS_pivotCANcoder.getConfigurator().apply(CS_pivotCANcoderConfig);
    }

    @Override
    public void updateInputs(ScorerIOInputs inputs) {
        inputs.CS_angle.mut_replace(CS_pivotCANcoder.getAbsolutePosition().getValue());
        inputs.CS_voltage.mut_replace(CS_pivotTalonFX.getMotorVoltage().getValue());
        inputs.CS_algae = CS_algaeSensor.get();
        inputs.CS_coral = CS_coralSensor.get();
    }

    @Override
    public void CS_runSetpoint(Angle angle) {
        CS_pivotTalonFX.setControl(new MotionMagicTorqueCurrentFOC(angle.in(Radians)).withSlot(0));
        CS_pivotTalonFX.setPosition(Units.radiansToRotations(angle.in(Radians)));
        Logger.recordOutput("Scorer/Pivot Setpoint", angle.in(Degrees));
    }

    @Override
    public void CS_setRoller(double speed) {
        CS_rollerTalonFX.set(speed);
        Logger.recordOutput("Scorer/Running", speed);
    }


}
