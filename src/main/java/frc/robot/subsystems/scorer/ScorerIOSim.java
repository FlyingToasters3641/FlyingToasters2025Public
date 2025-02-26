package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ScorerIOSim implements ScorerIO {
    private final ProfiledPIDController CS_PID_Controller = new ProfiledPIDController(
            ScorerConstants.CS_PROFILED_PID_CONSTANTS.kP, ScorerConstants.CS_PROFILED_PID_CONSTANTS.kI,
            ScorerConstants.CS_PROFILED_PID_CONSTANTS.kD, ScorerConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    private final ArmFeedforward CS_FeedForward = new ArmFeedforward(0, 0, 0, 0);
    private MutVoltage CS_appliedVoltage = Volts.mutable(0);
    private final SimDevice CS_algaeDevice = SimDevice.create("AlgaeSwitchDevice");
    private final SimBoolean CS_algaeSwitch = CS_algaeDevice.createBoolean("AlgaeSwitch", Direction.kInput, false);
    private final SimDevice CS_coralDevice = SimDevice.create("CoralSwitchDevice");
    private final SimBoolean CS_coralSwitch = CS_coralDevice.createBoolean("CoralSwitch", Direction.kInput, false);

    private final SingleJointedArmSim CS_ARM_sim = new SingleJointedArmSim(
            ScorerConstants.kArmPlant,
            ScorerConstants.kKrakenX60Foc,
            ScorerConstants.kArmGearing,
            ScorerConstants.kArmLength,
            ScorerConstants.kArmMinAngle,
            ScorerConstants.kArmMaxAngle,
            ScorerConstants.kSimulateGravity,
            ScorerConstants.kArmStartAngle);

    public ScorerIOSim() {

    }

    @Override
    public void updateInputs(ScorerIOInputs inputs) {
        CS_ARM_sim.update(.02);
        inputs.CS_angle.mut_replace(CS_ARM_sim.getAngleRads(), Radians);
        inputs.CS_voltage.mut_replace(CS_appliedVoltage);
        inputs.CS_setpointAngle.mut_replace(this.CS_PID_Controller.getGoal().position, Degrees);
        inputs.CS_algae = CS_algaeSwitch.get();
        inputs.CS_coral = CS_coralSwitch.get();
    }

    @Override
    public void CS_runSetpoint(Angle setpoint) {
        Angle currentAngle = Radians.of(CS_ARM_sim.getAngleRads());
        Logger.recordOutput("Scorer/Setpoint", setpoint.in(Degrees));
        Angle setpointAngle = Degrees.of(CS_PID_Controller.getSetpoint().position);
        AngularVelocity setpointVelocity = DegreesPerSecond.of(CS_PID_Controller.getSetpoint().velocity);
        Logger.recordOutput("Scorer/CurrentAngle", currentAngle.in(Degrees));
        Voltage controllerVoltage = Volts
                .of(CS_PID_Controller.calculate(currentAngle.in(Degrees), setpoint.in(Degrees)));

        // Voltage feedForwardVoltage =
        // Volts.of(CS_FeedForward.calculate(setpointAngle.in(Radians),
        // setpointVelocity.in(RadiansPerSecond)));

        CS_runVolts(controllerVoltage);
    }

    @Override
    public void CS_runVolts(Voltage volts) {
        double appliedVoltage = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
        CS_appliedVoltage.mut_replace(appliedVoltage, Volts);
        CS_ARM_sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void CS_setRoller(double speed) {
        Logger.recordOutput("Scorer/Running", speed);
    }

    @Override
    public void CS_reverseScorer(boolean reverse) {
    }

    @Override
    public void CS_setPID(double p, double i, double d) {
        CS_PID_Controller.setPID(p, i, d);
    }

    @Override
    public void CS_shootSimAlgae(AbstractDriveTrainSimulation driveSimulation) {
        ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(Constants.scorerPoseOffset.getX(), Constants.scorerPoseOffset.getY()),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSimulation.getSimulatedDriveTrainPose().getRotation()
                                .plus(new Rotation2d(Units.Degrees.of(45).in(Radians))),
                        ScorerConstants.CS_SCORE_NET_DISTANCE, // initial height of the ball, in meters
                        ScorerConstants.CS_SCORE_NET_VELOCITY, // initial velocity, in m/s
                        ScorerConstants.CS_SCORE_NET) // shooter angle
                        .withProjectileTrajectoryDisplayCallBack(
                                (poses) -> Logger.recordOutput("successfulShotsTrajectory",
                                        poses.toArray(Pose3d[]::new)),
                                (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new))));

    }

    @Override
    public void CS_shootL1SimCoral(AbstractDriveTrainSimulation driveSimulation){
        SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.35, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation().plus(new Rotation2d(Degrees.of(270).in(Radians))),
            // The height at which the coral is ejected
            Meters.of(0.5),
            // The initial speed of the coral
            MetersPerSecond.of(2),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-90)));
    }

    @Override
    public void CS_shootL2SimCoral(AbstractDriveTrainSimulation driveSimulation){
        SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.35, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation().plus(new Rotation2d(Degrees.of(270).in(Radians))),
            // The height at which the coral is ejected
            Meters.of(0.9),
            // The initial speed of the coral
            MetersPerSecond.of(2),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-35)));
    }
    


    @Override
    public void CS_shootL3SimCoral(AbstractDriveTrainSimulation driveSimulation) {
        SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.35, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation().plus(new Rotation2d(Degrees.of(270).in(Radians))),
            // The height at which the coral is ejected
            Meters.of(1.28),
            // The initial speed of the coral
            MetersPerSecond.of(2),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-35)));
    }

    @Override
    public void CS_shootL4SimCoral(AbstractDriveTrainSimulation driveSimulation){
        SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.46, 0), //-0.7, -0.51
        // Obtain robot speed from drive simulation
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getRotation().plus(new Rotation2d(Degrees.of(270).in(Radians))),
        // The height at which the coral is ejected
        Meters.of(2.1),
        // The initial speed of the coral
        MetersPerSecond.of(1),
        // The coral is ejected vertically downwards
        Degrees.of(-90)));
    }



}