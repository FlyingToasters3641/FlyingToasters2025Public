package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class IntakeCommands {
    public static Command IN_setRunning(Intake m_Intake, double speed) {
        return Commands.runOnce(() -> m_Intake.IN_setRoller(speed));
    }


    public static Command IN_runSetpoint(Intake m_Intake, Angle angle) {
        return  m_Intake.IN_runSetpoint(angle);
    }

    public static Command IN_intakeAlgae(Intake m_Intake){
        return Commands.parallel(m_Intake.IN_runSetpoint(Degrees.of(-45)),(IN_setRunning(m_Intake, 1.0)));
    }

    public static Command IN_intakeCoral(Intake m_Intake){
        return m_Intake.IN_runSetpoint(Degrees.of(160)).alongWith(IN_setRunning(m_Intake, 0.75));
    }

    public static Command IN_outakeAlgae(Intake m_Intake){
        return IN_rest(m_Intake).alongWith(IN_setRunning(m_Intake, -1.0));
    }

    public static Command IN_clearElevator(Intake m_Intake){
        return m_Intake.IN_runSetpoint(Degrees.of(-20));
    }

    public static Command IN_rest(Intake m_Intake){
        return m_Intake.IN_runSetpoint(Degrees.of(0)).alongWith(IN_stop(m_Intake));
    }

    public static Command IN_stop(Intake m_Intake){
        return Commands.runOnce(() -> m_Intake.IN_setRoller(0.0));
    }

    public static Command IN_simSetRunning(Intake m_Intake){
        return Commands.runOnce(() -> m_Intake.IN_setRunning(true));
    }
    
    public static Command IN_simStopRunning(Intake m_Intake){
        return Commands.runOnce(() -> m_Intake.IN_setRunning(false));
    }

}
