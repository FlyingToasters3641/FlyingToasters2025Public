package frc.robot.subsystems.intake;

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

public class IntakeIOTalonFX implements IntakeIO{
    DigitalInput coralSensor = new DigitalInput(0);
    DigitalInput algaeSensor = new DigitalInput(1);
    private static final String CANbusName = "scorer";
    public static final TalonFX IN_pivotTalonFX = new TalonFX(1, CANbusName);
    public static final TalonFX IN_rollerTalonFX = new TalonFX(2, CANbusName);
    public static final CANcoder IN_pivotCANcoder = new CANcoder(6, CANbusName);

    /**
     * 
     */
    public IntakeIOTalonFX(){

        /*
         * CS_pivotConfig is a TalonFXConfiguration object that holds all of the settings for the pivot motor.
         */
        TalonFXConfiguration IN_pivotConfig = new TalonFXConfiguration();

        IN_pivotConfig.Feedback.FeedbackRemoteSensorID = IN_pivotTalonFX.getDeviceID();
        IN_pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        IN_pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.IN_pivotConfig.SENSOR_TO_MECHANISM_RATIO;
        IN_pivotConfig.Feedback.RotorToSensorRatio = IntakeConstants.IN_pivotConfig.ROTOR_TO_SENSOR_RATIO;

        IN_pivotConfig.Slot0.kG = IntakeConstants.IN_pivotConfig.kG;
        IN_pivotConfig.Slot0.kP = IntakeConstants.IN_pivotConfig.kP;
        IN_pivotConfig.Slot0.kI = IntakeConstants.IN_pivotConfig.kI;
        IN_pivotConfig.Slot0.kD = IntakeConstants.IN_pivotConfig.kD;

        IN_pivotConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicCruiseVelocity(
            Units.radiansToRotations(IntakeConstants.IN_pivotConfig.MM_CRUISE_VELOCITY))
            .withMotionMagicAcceleration(IntakeConstants.IN_pivotConfig.MM_ACCELERATION);

            IN_pivotTalonFX.getConfigurator().apply(IN_pivotConfig);

        /*
         * CS_pivotCANcoderConfig is a TalonFXConfiguration object that holds all of the settings for the CANcoder on the pivot motor.
         */
        CANcoderConfiguration CS_pivotCANcoderConfig = new CANcoderConfiguration();
        CS_pivotCANcoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.IN_pivotConfig.PIVOT_MAGNET_OFFSET;
        
        IN_pivotCANcoder.getConfigurator().apply(CS_pivotCANcoderConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.IN_angle.mut_replace(IN_pivotCANcoder.getAbsolutePosition().getValue());
        inputs.IN_voltage.mut_replace(IN_pivotTalonFX.getMotorVoltage().getValue());
    }

    @Override
    public void IN_runSetpoint(Angle angle) {
        IN_pivotTalonFX.setControl(new MotionMagicTorqueCurrentFOC(angle.in(Radians)).withSlot(0));
        IN_pivotTalonFX.setPosition(Units.radiansToRotations(angle.in(Radians)));
        Logger.recordOutput("Scorer/Pivot Setpoint", angle.in(Degrees));
    }

    @Override
    public void IN_setRoller(double speed) {
        IN_rollerTalonFX.set(speed);
        Logger.recordOutput("Scorer/Running", speed);
    }


}
