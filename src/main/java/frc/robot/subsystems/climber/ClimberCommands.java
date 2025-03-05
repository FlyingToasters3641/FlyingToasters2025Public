package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class ClimberCommands {

    public static Command CL_setRotation (Climber m_Climber, Angle m_rotation){
        return Commands.run(() -> m_Climber.CL_setpoint = m_rotation);
    }

    public static Command CL_stop (Climber m_Climber){
        return Commands.run(() -> m_Climber.CL_Stop());
    }

    public static Command CL_setPosition (Climber m_Climber, double m_position){
        return Commands.run(() -> m_Climber.CL_setPosition(m_position));
    }

    public static Command CL_Extend(Climber m_Climber){
        return Commands.run(() -> m_Climber.CL_setServo(ClimberConstants.SERVO_RATCHET_DISENGAGED.in(Degrees)))
        .andThen(new WaitCommand(Time.ofBaseUnits(0.5, Second)))
        .andThen(() -> m_Climber.CL_setPosition(ClimberConstants.EXTENDED_POSITION));
    }

    public static Command CL_Retract(Climber m_Climber){
        return Commands.run(() -> m_Climber.CL_setServo(ClimberConstants.SERVO_RATCHET_ENGAGED.in(Degrees)))
        .andThen(() -> m_Climber.CL_setPosition(ClimberConstants.RETRACTED_POSITION));
    }

    public static Command CL_testSpeed(Climber m_Climber, DoubleSupplier xSupplier){
        return Commands.run(() -> m_Climber.CL_setSpeed(xSupplier.getAsDouble()));
    }

    public static Command CL_setServo(Climber m_Climber, double Angle) {
        return Commands.runOnce(() -> m_Climber.CL_setServo(Angle));
    }

    //Commange to lower the climber into the ENGAGED position
    public static SequentialCommandGroup CL_home(Climber m_Climber) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> m_Climber.CL_setSpeed(-0.2)).alongWith(new WaitCommand(0.5)),
            Commands.run(() -> m_Climber.CL_setSpeed(-0.2)).until(() -> m_Climber.CL_currentVelocity >= ClimberConstants.STALL_VELOCITY_THRESHOLD),
            Commands.runOnce(() -> m_Climber.CL_Home())
        );
    }
}
