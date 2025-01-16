package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

public class ElevatorIOTalonFX implements ElevatorIO {

    // TODO: Calculate Elevator Ratios
    public static final double EL_RATIO = 0.0;
    public static final double EL_ENCODER_RATIO = 0.0;

    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    public static final TalonFX EL_TalonFXOne = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    public static final TalonFX EL_TalonFXTwo = new TalonFX(2, CANbusName); // TODO: Standardize Names for Motors
    public static final CANcoder EL_CANCoder = new CANcoder(5, CANbusName); // TODO: Clarify CANcoder usage

    MutDistance setpoint = Inches.mutable(0);

    public ElevatorIOTalonFX() {

        // Assuming all motors run in the same direction, sets all motors to "follow"
        // EL_TalonFXOne
        EL_TalonFXTwo.setControl(new Follower(EL_TalonFXOne.getDeviceID(), false));

        TalonFXConfiguration EL_TalonConfig = new TalonFXConfiguration();

        // TODO: Edit Elevator Config Settings
        EL_TalonConfig.Slot0.kP = 0.0;
        EL_TalonConfig.Slot0.kD = 0.0;
        EL_TalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Measuring Feedback
        EL_TalonConfig.Feedback.FeedbackRemoteSensorID = EL_CANCoder.getDeviceID();
        EL_TalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        EL_TalonConfig.Feedback.SensorToMechanismRatio = EL_ENCODER_RATIO;
        EL_TalonConfig.Feedback.RotorToSensorRatio = EL_RATIO;

        // Ported same MotionMagic code from 2024
        EL_TalonConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(0);

        EL_TalonFXOne.getConfigurator().apply(EL_TalonConfig);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){;
        inputs.EL_position.mut_replace(Inches.of(EL_CANCoder.getPosition().getValueAsDouble()));
        Logger.recordOutput("Elevator/absolute", EL_CANCoder.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorOnePos", EL_TalonFXOne.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorTwoPos", EL_TalonFXTwo.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/voltage", EL_TalonFXOne.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    @Override
    public void setELPosition(Distance position){
        setpoint.mut_replace(position);
        EL_TalonFXOne.setControl(new MotionMagicTorqueCurrentFOC(position.copy().in(Inches)).withSlot(0)); //May need to make position negative if needed.
        //TODO: this may be a thing to be debugged but WPILib has had issues with mutable Units randomly being changed. -AU
    }

}
