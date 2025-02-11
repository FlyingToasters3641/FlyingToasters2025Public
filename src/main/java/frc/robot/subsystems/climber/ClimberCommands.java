package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class ClimberCommands {

    public static Command CL_setRotation (Climber m_Climber, Angle m_rotation){
        return Commands.run(() -> m_Climber.CL_setpoint = m_rotation);
    }

    public static Command CL_stop (Climber m_Climber){
        return Commands.run(() -> m_Climber.CL_Stop());
    }

    public static Command CL_home (Climber m_Climber){
        return Commands.run(() -> m_Climber.CL_Home());
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
}
