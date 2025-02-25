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

    public static final double EL_RATIO = 5.0;
    public static final double EL_ENCODER_RATIO = (1/(2.89*Math.PI));
    public static final double absoluteEncoderOffset = 0.326660;

    private static final String CANbusName = "maximo";
    //Master Motor
    public static final TalonFX EL_Left = new TalonFX(15, CANbusName);
    //Follower Motor
    public static final TalonFX EL_Right = new TalonFX(17, CANbusName); 
    public static final CANcoder EL_CANCoderLeft = new CANcoder(20, CANbusName); 
    public static final CANcoder EL_CANCoderRight = new CANcoder(19, CANbusName); 

    private static TalonFXConfiguration EL_TalonConfig = new TalonFXConfiguration();
    private static TalonFXConfiguration EL_TalonConfigSlow = new TalonFXConfiguration();

    MutDistance setpoint = Inches.mutable(0);

    public ElevatorIOTalonFX() {
        EL_Right.setControl(new Follower(EL_Left.getDeviceID(), true));

        //TWO PID SLOT LOOPS
        EL_TalonConfig.Slot0.kP = 35.0;
        EL_TalonConfig.Slot0.kD = 1.0;
        EL_TalonConfigSlow.Slot0.kP = 3.0;
        EL_TalonConfigSlow.Slot0.kD = 1.0;

        //Set Motors Inverted 
        EL_TalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        EL_TalonConfigSlow.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Measuring Feedback
        EL_TalonConfig.Feedback.FeedbackRemoteSensorID = EL_CANCoderLeft.getDeviceID();
        EL_TalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        EL_TalonConfig.Feedback.SensorToMechanismRatio = EL_ENCODER_RATIO;
        EL_TalonConfig.Feedback.RotorToSensorRatio = EL_RATIO;

        EL_TalonConfigSlow.Feedback.FeedbackRemoteSensorID = EL_CANCoderLeft.getDeviceID();
        EL_TalonConfigSlow.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        EL_TalonConfigSlow.Feedback.SensorToMechanismRatio = EL_ENCODER_RATIO;
        EL_TalonConfigSlow.Feedback.RotorToSensorRatio = EL_RATIO;

        // Motion Magic
        EL_TalonConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(1000)
                .withMotionMagicCruiseVelocity(200);
        EL_TalonConfigSlow.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(200)
                .withMotionMagicCruiseVelocity(50);

        CANcoderConfiguration magConfig = new CANcoderConfiguration();
        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        magConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        EL_CANCoderLeft.getConfigurator().apply(magConfig);
        EL_Left.getConfigurator().apply(EL_TalonConfig);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){;
        inputs.EL_position.mut_replace(Inches.of(EL_CANCoderLeft.getPosition().getValueAsDouble()));
        inputs.EL_voltage.mut_replace(Volt.of(EL_Left.getMotorVoltage().getValueAsDouble()));

        //Logs from TalonFX simulation.
        Logger.recordOutput("Elevator/motorOneVoltage", EL_Left.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/motorOnePos", EL_Left.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/motorTwoPos", EL_Right.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/voltage", EL_Left.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    @Override
    public void setELPosition(double position){
        setpoint.mut_replace(position, Inches);

        //Run logic to estimate whether the elevator is moving up or down
        if (position < EL_CANCoderLeft.getPosition().getValueAsDouble()){
            EL_Left.getConfigurator().apply(EL_TalonConfigSlow);
        } else {
            EL_Left.getConfigurator().apply(EL_TalonConfig);
        }

        Logger.recordOutput("Elevator/SetpointPositionCall", position);
        EL_Left.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(0));

    }

    @Override
    public void EL_setSpeed(double speed){
        EL_Left.set(speed);

        Logger.recordOutput("Elevator/Speed", speed);
    }



    
    

}
