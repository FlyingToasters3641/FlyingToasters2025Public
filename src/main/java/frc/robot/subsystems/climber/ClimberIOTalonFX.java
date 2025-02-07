package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.MutAngle;

public class ClimberIOTalonFX implements ClimberIO {

    public static final double CL_RATIO = 0.0;
    public static final double CL_ENCODER_RATIO = 0.0;

    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    public static final TalonFX CL_TalonFX = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    public static final CANcoder CL_CANCoder = new CANcoder(5, CANbusName); // TODO: Clarify CANcoder usage

    MutAngle setpoint = Radians.mutable(0);

    public ClimberIOTalonFX() {

        TalonFXConfiguration CL_TalonConfig = new TalonFXConfiguration();

        CL_TalonConfig.Slot0.kP = 0.0;
        CL_TalonConfig.Slot0.kD = 0.0;
        CL_TalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Measuring Feedback
        CL_TalonConfig.Feedback.FeedbackRemoteSensorID = CL_CANCoder.getDeviceID();
        CL_TalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        CL_TalonConfig.Feedback.SensorToMechanismRatio = CL_ENCODER_RATIO;
        CL_TalonConfig.Feedback.RotorToSensorRatio = CL_RATIO;

        // Ported same MotionMagic code from 2024
        CL_TalonConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(0);

        CL_TalonFX.getConfigurator().apply(CL_TalonConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){;
        inputs.CL_rotation.mut_replace(Radians.of(CL_CANCoder.getPosition().getValueAsDouble()));

        //Logs from TalonFX simulation.

        Logger.recordOutput("Climber/motorPos", CL_TalonFX.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/voltage", CL_TalonFX.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Climber/setpoint", setpoint);
    }

    @Override
    public void setCLRotation(double rotation){
        setpoint.mut_replace(rotation, Radians);
        CL_TalonFX.setControl(new MotionMagicTorqueCurrentFOC(rotation).withSlot(0)); //May need to make position negative if needed.
        //TODO: this may be a thing to be debugged but WPILib has had issues with mutable Units randomly being changed. -AU
    }
}
