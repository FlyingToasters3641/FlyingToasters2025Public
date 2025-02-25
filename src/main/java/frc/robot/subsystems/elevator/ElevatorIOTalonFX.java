package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.MutDistance;

public class ElevatorIOTalonFX implements ElevatorIO {

    // TODO: Calculate Elevator Ratios
    public static final double EL_RATIO = 5.0;
    public static final double EL_ENCODER_RATIO = (1/(2.89*Math.PI));
    public static final double absoluteEncoderOffset = 0.326660;

    private static final String CANbusName = "maximo"; // TODO: Update CANbus Name
    public static final TalonFX EL_TalonFXOne = new TalonFX(15, CANbusName);// TODO: Update CANIDs
    public static final TalonFX EL_TalonFXTwo = new TalonFX(17, CANbusName); // TODO: Standardize Names for Motors
    public static final CANcoder EL_CANCoder = new CANcoder(20, CANbusName); // TODO: Clarify CANcoder usage
    public static final CANcoder EL_CANCoderTwo = new CANcoder(19, CANbusName); // TODO: Clarify CANcoder usage


    MutDistance setpoint = Inches.mutable(0);

    public ElevatorIOTalonFX() {

        // Assuming all motors run in the same direction, sets all motors to "follow"
        // EL_TalonFXOne
        EL_TalonFXTwo.setControl(new Follower(EL_TalonFXOne.getDeviceID(), true));

        TalonFXConfiguration EL_TalonConfig = new TalonFXConfiguration();

        // TODO: Edit Elevator Config Settings
        EL_TalonConfig.Slot0.kP = 35.0;
        EL_TalonConfig.Slot0.kD = 1.0;
        EL_TalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Measuring Feedback
        EL_TalonConfig.Feedback.FeedbackRemoteSensorID = EL_CANCoder.getDeviceID();
        EL_TalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        EL_TalonConfig.Feedback.SensorToMechanismRatio = EL_ENCODER_RATIO;
        EL_TalonConfig.Feedback.RotorToSensorRatio = EL_RATIO;

        // Ported same MotionMagic code from 2024
        EL_TalonConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(1000)
                .withMotionMagicCruiseVelocity(200);

        CANcoderConfiguration magConfig = new CANcoderConfiguration();
        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        magConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        EL_CANCoder.getConfigurator().apply(magConfig);
        EL_TalonFXOne.getConfigurator().apply(EL_TalonConfig);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){;
        inputs.EL_position.mut_replace(Inches.of(EL_CANCoder.getPosition().getValueAsDouble()));
        inputs.EL_voltage.mut_replace(Volt.of(EL_TalonFXOne.getMotorVoltage().getValueAsDouble()));

        //Logs from TalonFX simulation.
        Logger.recordOutput("Elevator/motorOneVoltage", EL_TalonFXOne.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/motorOnePos", EL_TalonFXOne.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorTwoPos", EL_TalonFXTwo.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/voltage", EL_TalonFXOne.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    @Override
    public void setELPosition(double position){
        setpoint.mut_replace(position, Inches);
        Logger.recordOutput("Elevator/SetpointPositionCall", position);
        EL_TalonFXOne.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(0)); //May need to make position negative if needed.
        //TODO: this may be a thing to be debugged but WPILib has had issues with mutable Units randomly being changed. -AU
    }

    @Override
    public void EL_setSpeed(double speed){
        EL_TalonFXOne.set(speed);

        Logger.recordOutput("Elevator/Speed", speed);
    }



    
    

}
