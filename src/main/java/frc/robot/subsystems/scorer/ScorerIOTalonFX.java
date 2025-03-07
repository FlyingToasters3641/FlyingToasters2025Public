package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

public class ScorerIOTalonFX implements ScorerIO{
    private static final String CANbusName = "maximo";
    public static final TalonFX CS_pivotTalonFX = new TalonFX(16, CANbusName);
    public static final TalonFX CS_rollerTalonFX = new TalonFX(18, CANbusName);
    public static final CANcoder CS_pivotCANcoder = new CANcoder(21, CANbusName);
    public static final DigitalInput CS_coralSensor = new DigitalInput(0);
    public static final DigitalInput CS_algaeSensor = new DigitalInput(2);

    public ScorerIOTalonFX(){

        /*
         * CS_pivotConfig is a TalonFXConfiguration object that holds all of the settings for the pivot motor.
         */
        TalonFXConfiguration CS_pivotConfig = new TalonFXConfiguration();

        CS_pivotConfig.Feedback.FeedbackRemoteSensorID = CS_pivotCANcoder.getDeviceID();
        CS_pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        CS_pivotConfig.Feedback.SensorToMechanismRatio = ScorerConstants.CS_pivotConfig.SENSOR_TO_MECHANISM_RATIO;
        CS_pivotConfig.Feedback.RotorToSensorRatio = ScorerConstants.CS_pivotConfig.ROTOR_TO_SENSOR_RATIO;
        
        CS_pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CS_pivotConfig.Slot0.kG = ScorerConstants.CS_pivotConfig.kG;
        CS_pivotConfig.Slot0.kP = ScorerConstants.CS_pivotConfig.kP;
        CS_pivotConfig.Slot0.kI = ScorerConstants.CS_pivotConfig.kI;
        CS_pivotConfig.Slot0.kD = ScorerConstants.CS_pivotConfig.kD;
        CS_pivotConfig.Slot0.kV = ScorerConstants.CS_pivotConfig.kV;
        CS_pivotConfig.Slot1.kG = ScorerConstants.CS_pivotConfig.AkG;
        CS_pivotConfig.Slot1.kP = ScorerConstants.CS_pivotConfig.AkP;
        CS_pivotConfig.Slot1.kI = ScorerConstants.CS_pivotConfig.AkI;
        CS_pivotConfig.Slot1.kD = ScorerConstants.CS_pivotConfig.AkD;
        CS_pivotConfig.Slot1.kV = ScorerConstants.CS_pivotConfig.AkV;
        CS_pivotTalonFX.setNeutralMode(NeutralModeValue.Brake);

        CS_pivotConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicCruiseVelocity(
            Units.radiansToRotations(ScorerConstants.CS_pivotConfig.MM_CRUISE_VELOCITY))
            .withMotionMagicAcceleration(ScorerConstants.CS_pivotConfig.MM_ACCELERATION);

        CS_pivotTalonFX.getConfigurator().apply(CS_pivotConfig);

        /*
         * CS_pivotCANcoderConfig is a TalonFXConfiguration object that holds all of the settings for the CANcoder on the pivot motor.
         */
        CANcoderConfiguration CS_pivotCANcoderConfig = new CANcoderConfiguration();
        CS_pivotCANcoderConfig.MagnetSensor.MagnetOffset = ScorerConstants.CS_pivotConfig.PIVOT_MAGNET_OFFSET;
        CS_pivotCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        CS_pivotCANcoder.getConfigurator().apply(CS_pivotCANcoderConfig);
    }

    @Override
    public void updateInputs(ScorerIOInputs inputs) {
        inputs.CS_angle.mut_replace(CS_pivotCANcoder.getAbsolutePosition().getValue());
        inputs.CS_voltage.mut_replace(CS_pivotTalonFX.getMotorVoltage().getValue());
        inputs.CS_algae = CS_algaeSensor.get();
        inputs.CS_coral = CS_coralSensor.get();

        Logger.recordOutput("Scorer/MotorPosition", Units.rotationsToDegrees(CS_pivotTalonFX.getPosition().getValueAsDouble()));
        Logger.recordOutput("Scorer/CanCoderPosition", Units.rotationsToDegrees(CS_pivotCANcoder.getPosition().getValueAsDouble()));
        Logger.recordOutput("Scorer/AlgaeSensor", CS_algaeSensor.get());
    }

    @Override
    public void CS_runSetpoint(Angle angle) {
        CS_pivotTalonFX.setControl(new MotionMagicTorqueCurrentFOC(angle.in(Rotations)).withSlot(0));
        Logger.recordOutput("Scorer/Pivot Setpoint", angle.in(Degrees));
    }

    @Override
    public void CS_setRoller(double speed) {
        CS_rollerTalonFX.set(speed);
        Logger.recordOutput("Scorer/Running", speed);
    }

    @Override
    public void CS_setSpeed(double speed){
        CS_pivotTalonFX.set(speed);
        Logger.recordOutput("Scorer/PivotSpeed", speed);
    }

    @Override
    public boolean CS_getAlgaeSensor() {
        return CS_algaeSensor.get();
    }

    @Override
    public boolean CS_getCoralSensor() {
        return CS_coralSensor.get();
    }



}
