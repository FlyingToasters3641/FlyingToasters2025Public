package frc.robot.subsystems.intake;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {
    public static Command IN_setRunning(Intake m_Intake, boolean runIntake) {
        return Commands.run(() -> m_Intake.IN_setRunning(runIntake));
    }

    public static Command IN_reverseIntake(Intake m_Intake, boolean reverse) {
        return Commands.runOnce(() -> m_Intake.IN_reverseIntake(reverse));
    }
}
