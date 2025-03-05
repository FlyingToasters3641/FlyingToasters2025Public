package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOTalonFX implements ClimberIO {

    public static final double CL_RATIO = 0.0;
    public static final double CL_ENCODER_RATIO = 0.0;

    private static final String CANbusName = "maximo"; 
    public static final TalonFX CL_TalonFX = new TalonFX(14, CANbusName);
    public static final Servo CL_Servo = new Servo(9);

    MutAngle setpoint = Rotations.mutable(0);

    public ClimberIOTalonFX() {

        TalonFXConfiguration CL_TalonConfig = new TalonFXConfiguration();

        CL_TalonConfig.Slot0.kP = 0.1;
        CL_TalonConfig.Slot0.kD = 0.0;
        CL_TalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CL_TalonFX.getConfigurator().apply(CL_TalonConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){;

        //Logs from TalonFX simulation.
        inputs.CL_position = CL_TalonFX.getPosition().getValueAsDouble();
        inputs.CL_currentVelocity = CL_TalonFX.getVelocity().getValueAsDouble();
        Logger.recordOutput("Climber/velocity",  CL_TalonFX.getVelocity().getValueAsDouble());
        Logger.recordOutput("Climber/motorPos", CL_TalonFX.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/voltage", CL_TalonFX.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Climber/setpoint", setpoint);
        Logger.recordOutput("Climber/position", CL_TalonFX.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/servo", CL_Servo.getAngle());
    }

    @Override
    public void CL_setPosition(double position){
        setpoint.mut_replace(position, Rotations);
        CL_TalonFX.setControl(new MotionMagicTorqueCurrentFOC(position).withSlot(0)); 
    }

    @Override
    public void CL_Home(){
        CL_TalonFX.setPosition(0);
        CL_TalonFX.set(0.0);
    }

    @Override
    public void CL_setServo(double angle){
        CL_Servo.setAngle(angle);
    }

    @Override
    public void CL_Stop(){
        CL_TalonFX.set(0);
    }

    @Override
    public void CL_setSpeed(double speed){
        CL_TalonFX.set(speed);
    } 

    @Override
    public boolean CL_getExtended(){
        return CL_TalonFX.getPosition().getValueAsDouble() > 1000.0;
    }

    @Override
    public boolean CL_getServoDisengaged() {
        return CL_Servo.getAngle() > 45;
    }
}
