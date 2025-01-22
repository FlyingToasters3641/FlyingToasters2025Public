package frc.robot.subsystems.intake;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {
    
public static Command raiseJoint(Intake m_intake, Double speed) {
        return Commands.run(() -> m_intake.raiseJointMotor(speed));
    }
public static Command lowerJoint(Intake m_intake, Double speed) {
        return Commands.run(() -> m_intake.lowerJointMotor(speed));
    }
    public static Command runWheels(Intake m_intake, double speed) {
        return Commands.run(() -> m_intake.runWheelRollers(speed));
    }
    public static Command reverseWheels(Intake m_intake, double speed) {
        return Commands.run(() -> m_intake.reverseWheelMotor(speed));
    }
}
