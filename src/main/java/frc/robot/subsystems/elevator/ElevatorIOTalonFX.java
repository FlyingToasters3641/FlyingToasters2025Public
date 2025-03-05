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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

public class ElevatorIOTalonFX implements ElevatorIO {

    public static final double absoluteEncoderOffset = 0.326660;

    private static final String CANbusName = "maximo";
    //Master Motor
    public static final TalonFX EL_Left = new TalonFX(15, CANbusName);
    //Follower Motor
    public static final TalonFX EL_Right = new TalonFX(17, CANbusName);

    public static final CANcoder EL_CANCoderLeft = new CANcoder(20, CANbusName); 
    public static final CANcoder EL_CANCoderRight = new CANcoder(19, CANbusName); 

    MutDistance setpoint = Inches.mutable(0);

    public ElevatorIOTalonFX() {

        TalonFXConfiguration EL_TalonConfig = new TalonFXConfiguration();
        EL_Right.setControl(new Follower(EL_Left.getDeviceID(), true));

        //TWO PID SLOT LOOPS
        EL_TalonConfig.Slot0.kP = ElevatorConstants.EL_talonConfig.kP;
        EL_TalonConfig.Slot0.kD = ElevatorConstants.EL_talonConfig.kD;
        EL_TalonConfig.Slot1.kP = ElevatorConstants.EL_talonConfig.AkP;
        EL_TalonConfig.Slot1.kD = ElevatorConstants.EL_talonConfig.AkD;
        //Set Motors Inverted 
        EL_TalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Measuring Feedback
        EL_TalonConfig.Feedback.FeedbackRemoteSensorID = EL_CANCoderLeft.getDeviceID();
        EL_TalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        EL_TalonConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.EL_talonConfig.SENSOR_TO_MECHANISM_RATIO;
        EL_TalonConfig.Feedback.RotorToSensorRatio = ElevatorConstants.EL_talonConfig.ROTOR_TO_SENSOR_RATIO;

        // Motion Magic
        EL_TalonConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(ElevatorConstants.EL_talonConfig.MM_ACCELERATION)
                .withMotionMagicCruiseVelocity(ElevatorConstants.EL_talonConfig.MM_CRUISE_VELOCITY);
        CANcoderConfiguration magConfig = new CANcoderConfiguration();
        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        magConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        EL_CANCoderLeft.getConfigurator().apply(magConfig);
        EL_Left.getConfigurator().apply(EL_TalonConfig);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){;
        inputs.EL_position.mut_replace(Inches.of(EL_Left.getPosition().getValueAsDouble()));
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

        Logger.recordOutput("Elevator/SetpointPositionCall", position);
        if (position < EL_Left.getPosition().getValueAsDouble()) {
        EL_Left.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(1));
        } else {
        EL_Left.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(0));
        }

    }

    @Override
    public void EL_setSpeed(double speed){
        EL_Left.set(speed);

        Logger.recordOutput("Elevator/Speed", speed);
    }

    @Override
    public Distance EL_getPosition(){
        return Inches.of(EL_Left.getPosition().getValueAsDouble());
    }    

}
