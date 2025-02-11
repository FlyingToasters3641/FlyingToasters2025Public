// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFindToPath;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BehaviorTree.BehaviorTreeDebugger;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.trees.ControlTree;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.lib.BehaviorTree.trees.Stack;
import frc.robot.lib.BehaviorTree.trees.Targets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerIO;
import frc.robot.subsystems.scorer.ScorerIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private final Elevator elevator;
    private final Intake intake;
    private final Scorer scorer;
    private final Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    public static Blackboard blackboard = new Blackboard();
    public static Dashboard dashboard = new Dashboard();
    
    public static Stack stack = new Stack(blackboard);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    //Choose a target through dhasboard
    private final LoggedDashboardChooser<Targets> targetChooser;

    //starting Auto Pose for simulation
    private final Pose2d startingAutoPose = new Pose2d(7.628, 6.554, new Rotation2d(3.1415926535897932384));


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                        new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO() {});
                scorer = new Scorer(new ScorerIO() {});
                climber = new Climber(new ClimberIO() {});
                break;
                       
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, (startingAutoPose));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                Logger.recordOutput("FieldSimulation/Algae",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
                Logger.recordOutput("FieldSimulation/Coral",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]));

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation, SimulatedArena.getInstance(), blackboard));
                scorer = new Scorer(new ScorerIOSim());
                climber = new Climber(new ClimberIO() {});
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO() {});
                scorer = new Scorer(new ScorerIO() {});
                climber = new Climber(new ClimberIO() {});
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        //Set up the target chooser for the stack
        targetChooser = new LoggedDashboardChooser<>("Target Choices");
        targetChooser.addOption("A1", Targets.A1);
        targetChooser.addOption("Processor", Targets.PROCESSOR);
        targetChooser.addOption("G1", Targets.G1);
        targetChooser.addOption("L1", Targets.L1);
        targetChooser.addOption("test", Targets.TEST);


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        BehaviorTreeDebugger debugger = BehaviorTreeDebugger.getInstance();
        debugger.enableLogging(true); // Enable debugging
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        controller.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        //Gyro reset
        controller.start().onTrue(Commands.runOnce(() -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))).ignoringDisable(true));
        //Pathfind to a pose
        controller.b().whileTrue(new PathFindToPath(drive, () -> Constants.testPath));
        //Change the simulation state of the intake
        controller.x().onTrue(Commands.runOnce(() -> switchIntakeStateSim()));
        //Add items to the stack - for testing!
        controller.leftBumper().onTrue(Commands.runOnce(() -> addToStack()));
        //Execute the control tree
        controller.y().toggleOnTrue(new ControlTree(blackboard).execute());

        controller.rightTrigger(0.1).whileTrue(IntakeCommands.IN_setRunning(intake, true)).onFalse(IntakeCommands.IN_setRunning(intake, false));
        controller.leftTrigger(0.1).whileTrue(IntakeCommands.IN_reverseIntake(intake, true));
        controller.povDown().onTrue(ClimberCommands.CL_Extend(climber));
        controller.povUp().onTrue(ClimberCommands.CL_Retract(climber));
        dashboard.L1().whileTrue(ElevatorCommands.EL_goToL1(elevator)).onFalse(ElevatorCommands.EL_goToRest(elevator));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    //hardsets the intake state to true or false, used for simulation testing
    public void switchIntakeStateSim() {
        if (!blackboard.getBoolean("hasCoral") && !blackboard.getBoolean("hasAlgae")) {
                blackboard.set("hasCoral", true);
                blackboard.set("hasAlgae", true);
        } else {
                blackboard.set("hasCoral", false);
                blackboard.set("hasAlgae", false);
        }
        Logger.recordOutput("hasCoral", blackboard.getBoolean("hasCoral"));
    }
    
    //Adds item for the stack - testing for the control tree
    public void addToStack() {
        Targets targetValue = targetChooser.get();
        stack.add(targetValue);
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(startingAutoPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }


    public void displayComponentPosesToAdvantageScope(){
        if(Constants.currentMode != Constants.Mode.SIM) return;

        Distance EL_simPosition = elevator.getELPosition();

        Transform3d elevator3d = new Transform3d(Inches.zero(), Inches.zero(), EL_simPosition, new Rotation3d(0,0,0)); //3d view changed to be a straight line up.
        Transform3d elevatorHalf3d = new Transform3d(Inches.zero(), Inches.zero(), EL_simPosition.div(2), new Rotation3d(0,0,0));

        Rotation3d scorerRotation3d = new Rotation3d(scorer.getCSAngle().in(Radians),0,0);

        Logger.recordOutput("Odometry/RobotComponentPoses", new Pose3d[] {new Pose3d(Constants.scorerPoseOffset.getX()+elevator3d.getX(),Constants.scorerPoseOffset.getY()+elevator3d.getY(),Constants.scorerPoseOffset.getZ()+elevator3d.getZ(),Constants.scorerPoseOffset.getRotation().rotateBy(scorerRotation3d)), new Pose3d(Constants.scorerRollerPoseOffset.getX()+elevator3d.getX(), Constants.scorerRollerPoseOffset.getY()+elevator3d.getY(), Constants.scorerRollerPoseOffset.getZ()+elevator3d.getZ(), Constants.scorerRollerPoseOffset.getRotation().rotateBy(scorerRotation3d)), Constants.climberPoseOffset, Constants.coralIntakePoseOffset, Constants.coralIntakeRollerPoseOffset, Constants.elevatorOneIntakeOffset.plus(elevator3d), Constants.elevatorTwoIntakeOffset.plus(elevatorHalf3d)});


    }

}
