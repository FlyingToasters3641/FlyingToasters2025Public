package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class IntakeCommands {
    public static Command IN_setRunning(Intake m_Intake, double speed) {
        return Commands.run(() -> m_Intake.IN_setRoller(speed));
    }

    public static Command IN_reverseIntake(Intake m_Intake, double speed, boolean reverse) {
        return Commands.runOnce(() -> m_Intake.IN_reverseIntake(speed, reverse));
    }

    public static Command IN_runSetpoint(Intake m_Intake, Angle angle) {
        return  m_Intake.IN_runSetpoint(angle);
    }

    public static Command IN_intakeCoral(Intake m_Intake, double speed){
        return Commands.runOnce(() -> IN_setRunning(m_Intake, speed));
    }

    public static Command IN_intakeAlgae(Intake m_Intake, double speed, boolean reverse){
        return Commands.runOnce(() -> IN_reverseIntake(m_Intake, -speed, reverse));
    }

    public static Command IN_clearElevator(Intake m_Intake){
        return m_Intake.IN_runSetpoint(Degrees.of(-20));
    }

    public static Command IN_rest(Intake m_Intake){
        return m_Intake.IN_runSetpoint(Degrees.of(0));
    }
}
