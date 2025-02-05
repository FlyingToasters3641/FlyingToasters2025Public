package frc.robot.subsystems.intake;



import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class IntakeCommands {
    public static Command IN_setRunning(Intake m_Intake, double speed) {
        return Commands.run(() -> m_Intake.IN_setRoller(speed));
    }
    public static Command IN_runSetangle(Intake m_Intake, Angle angle) {
        return Commands.runOnce(() -> m_Intake.IN_runSetpoint(angle));
    }
    public static Command IN_reverseIntake(Intake m_Intake, boolean reverse) {
        return Commands.runOnce(() -> m_Intake.IN_reverseIntake(reverse));
    }
}
