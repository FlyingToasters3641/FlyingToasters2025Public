package frc.robot.subsystems.elevator;

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

public class ElevatorIOTalonFX implements ElevatorIO {

    // TODO: Calculate Elevator Ratios
    public static final double EL_RATIO = 0.0;
    public static final double EL_ENCODER_RATIO = 0.0;

    private static final String CANbusName = "idk"; // TODO: Update CANbus Name
    public static final TalonFX EL_TalonFXOne = new TalonFX(1, CANbusName);// TODO: Update CANIDs
    public static final TalonFX EL_TalonFXTwo = new TalonFX(2, CANbusName);
    public static final TalonFX EL_TalonFXThree = new TalonFX(3, CANbusName);
    public static final TalonFX EL_TalonFXFour = new TalonFX(4, CANbusName); // TODO: Standardize Names for Motors
    public static final CANcoder EL_CANCoder = new CANcoder(5, CANbusName); // TODO: Clarify CANcoder usage

    double setpoint = 0.0;

    public ElevatorIOTalonFX() {

        // Assuming all motors run in the same direction, sets all motors to "follow"
        // EL_TalonFXOne
        EL_TalonFXTwo.setControl(new Follower(EL_TalonFXOne.getDeviceID(), false));
        EL_TalonFXThree.setControl(new Follower(EL_TalonFXOne.getDeviceID(), false));
        EL_TalonFXFour.setControl(new Follower(EL_TalonFXOne.getDeviceID(), false));

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
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.EL_position_inches = setpoint;
        inputs.EL_position = EL_CANCoder.getPosition().getValueAsDouble();
        Logger.recordOutput("Elevator/absolute", EL_CANCoder.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorOnePos", EL_TalonFXOne.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorTwoPos", EL_TalonFXTwo.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorThreePos", EL_TalonFXThree.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorFourPos", EL_TalonFXFour.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/voltage", EL_TalonFXOne.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    @Override
    public void setELPosition(double position){
        setpoint = position;
        EL_TalonFXOne.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(0)); //May need to make position negative if needed.
    }

}
