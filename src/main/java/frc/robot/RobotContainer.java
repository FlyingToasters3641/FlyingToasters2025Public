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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
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
import frc.robot.commands.ScoreCommands.NetTest.ScoreTarget;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BehaviorTree.BehaviorTreeDebugger;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerCommands;
import frc.robot.subsystems.scorer.ScorerIO;
import frc.robot.subsystems.scorer.ScorerIOSim;
import frc.robot.subsystems.scorer.ScorerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision2D;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim2D;

public class RobotContainer {
    // Subsystems
    public static Vision vision;
    public static Drive drive;
    public static Elevator elevator;
    public static Intake intake;
    public static Scorer scorer;
    public static Climber climber;
    public SwerveDriveSimulation driveSimulation = null;
    public static Blackboard blackboard = new Blackboard();
    public static Dashboard dashboard = new Dashboard();
    
    public static Stack stack = new Stack(blackboard);

    // Controller
    private static CommandXboxController operatorController = new CommandXboxController(1);
    private static CommandXboxController driverController = new CommandXboxController(0);

    // Dashboard inputs
    private static LoggedDashboardChooser<Command> autoChooser;
    //Choose a target through dhasboard
    private static LoggedDashboardChooser<Targets> targetChooser;
    //Choose a player station through dashboard
    private final LoggedDashboardChooser<String> playerStationChooser;

    //starting Auto Pose for simulation
    private static Pose2d startingAutoPose = new Pose2d(7.628, 6.554, new Rotation2d(3.1415926535897932384));


    /** The container for the robot. Contains subsystems, IO devices, and commands. */
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
                        new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                        new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                        new VisionIOPhotonVision(VisionConstants.camera3Name, VisionConstants.robotToCamera3),
                        new VisionIOPhotonVision2D(VisionConstants.camera4Name, VisionConstants.robotToCamera4),
                        new VisionIOPhotonVision2D(VisionConstants.camera5Name, VisionConstants.robotToCamera5));

                elevator = new Elevator(new ElevatorIOTalonFX() {});
                intake = new Intake(new IntakeIO() {});
                scorer = new Scorer(new ScorerIOTalonFX() {});
                climber = new Climber(new ClimberIOTalonFX() {});
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
                                VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, false),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name, VisionConstants.robotToCamera1, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, false),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera2Name, VisionConstants.robotToCamera2, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, false),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera3Name, VisionConstants.robotToCamera3, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, false),
                        new VisionIOPhotonVisionSim2D(
                                VisionConstants.camera4Name, VisionConstants.robotToCamera4, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, true),
                        new VisionIOPhotonVisionSim2D(
                                VisionConstants.camera5Name, VisionConstants.robotToCamera5, driveSimulation::getSimulatedDriveTrainPose, drive::getRotation, true));
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation, SimulatedArena.getInstance(), blackboard));
                scorer = new Scorer(new ScorerIOSim(), driveSimulation);
                climber = new Climber(new ClimberIO() {});
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO() {});
                scorer = new Scorer(new ScorerIO() {}, driveSimulation);
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
        targetChooser.addOption("B1", Targets.B1);
        targetChooser.addOption("B3", Targets.B3);
        targetChooser.addOption("B4", Targets.B4);
        targetChooser.addOption("B2", Targets.B2);

        playerStationChooser = new LoggedDashboardChooser<>("Human Player Station Choice:");
        playerStationChooser.addOption("Left", "left");
        playerStationChooser.addOption("Right", "right");




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
                drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -driverController.getRightTriggerAxis()
                ));

        // Switch to X pattern when X button is pressed
        //operatorController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        //operatorController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        //Gyro reset
        driverController.start().onTrue(Commands.runOnce(() -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))).ignoringDisable(true));
        //Score coral commands
        
        // driverController.b().or(dashboard.L4()).onTrue(new ScoreL4(scorer, elevator));
        // controller.y().or(dashboard.L3()).onTrue(new ScoreL3(scorer, elevator));
        // driverController.a().or(dashboard.L3()).onTrue(new ScoreL3(scorer, elevator));
        // driverController.x().or(dashboard.L2()).onTrue(new ScoreL2(scorer, elevator));
        // driverController.povRight().or(dashboard.L1()).onTrue(new ScoreL1(scorer, elevator));

        // Auto Align
        driverController.leftBumper().whileTrue(DriveCommands.sequentialAutoAlign(drive, () -> vision.robotLeftXOffsetToAprilTag(), () -> vision.robotLeftYOffsetToAprilTag(),() -> true));
        driverController.rightBumper().whileTrue(DriveCommands.sequentialAutoAlign(drive, () -> vision.robotRightXOffsetToAprilTag(), () -> vision.robotRightYOffsetToAprilTag(),() -> false));
        //driverController.leftBumper().whileTrue(DriveCommands.xyAlign(drive, () -> vision.getRobotLeftAprilTagSize(), () -> vision.getRobotLeftPitchAngle(), () -> true));
        //driverController.rightBumper().whileTrue(DriveCommands.xyAlign(drive, () -> vision.getRobotRightAprilTagSize(), () -> vision.getRobotRightPitchAngle(), () -> false));



        //driverController.rightBumper().onTrue(Commands.runOnce(() -> setTreeTarget()));
        //Score net
        //operatorController.rightBumper().or(dashboard.NET()).onTrue(new ScoreNet(scorer, elevator, intake));

        //Controls using blackboard values
        //
        
        driverController.b().onTrue(ElevatorCommands.EL_setPositionToBlackboard(elevator, blackboard)).onFalse(ElevatorCommands.EL_goToRest(elevator));
        //driverController.a().onTrue(Commands.sequence(IntakeCommands.IN_setPivotToBlackboard(intake, blackboard),IntakeCommands.IN_setSpeedToBlackboard(intake, blackboard)));
        driverController.x().onTrue(new ScoreTarget(scorer,elevator,blackboard));
        driverController.y().onTrue(Commands.runOnce(() -> setTreeTarget()));

        //Intake algae
        // operatorController.leftTrigger(0.1).onTrue(new IntakeGroundAlgae(scorer, intake));

        //Outake algae
        // operatorController.leftBumper().onTrue(new RemoveAlgae(scorer, intake));

        // //Intake coral
        // operatorController.rightTrigger(0.1).onTrue(new IntakeCoral(scorer, intake));

        //Climber controls
        // controller.povUp().onTrue(new StartClimb(scorer, elevator, intake));
        // controller.povDown().onTrue(new EndClimb(scorer));

        //Manual Climbing Commands
        /*
         * controller.povUp().onTrue(new StartClimbTest());
         * controller.povDown().onTrue(new EndClimberTest());
         * 
         */
        //operatorController.axisGreaterThan(5,0.1).onTrue(ClimberCommands.CL_testSpeed(climber, () -> operatorController.getRightY()));
        //Engages the small ratchet on the side by setting the position to 0.0
        //operatorController.povLeft().toggleOnTrue(new ConditionalCommand(ClimberCommands.CL_setServo(climber, 0), ClimberCommands.CL_setServo(climber, 90), () -> climber.CL_getServoDisengaged()));
        //operatorController.povUp().onTrue(ClimberCommands.CL_home(climber));

        //Elevator Manual Calibration
        //operatorController.axisGreaterThan(1,0.1).onTrue(ElevatorCommands.EL_joystickControl(elevator,() -> operatorController.getLeftY()));
        //LEVEL 4 SCORING
        operatorController.povUp().onTrue(ElevatorCommands.EL_setPosition(elevator, Inches.of(52)));
        operatorController.povDown().onTrue(ElevatorCommands.EL_setPosition(elevator, Inches.of(2)));
        //LEVEL 2 SCORING
        operatorController.povRight().onTrue(ElevatorCommands.EL_setPosition(elevator, Inches.of(7)));
        //LEVEL 3 SCORING
        operatorController.povLeft().onTrue(ElevatorCommands.EL_setPosition(elevator, Inches.of(22)));

        operatorController.leftBumper().onTrue(ScoreCommands.NetTest(scorer, elevator));
        operatorController.rightBumper().onTrue(ScoreCommands.L2AlgaeSequence(scorer, elevator));
        
        //Scorer Manual Calibration (uncomment when needed)
        //operatorController.axisGreaterThan(1, 0.1).onTrue(ScorerCommands.CS_joystickControl(scorer, () -> operatorController.getLeftY()));
        
        operatorController.a().onTrue(ScorerCommands.CS_runSetpoint(scorer, Degrees.of(0)));
        operatorController.x().onTrue(ScorerCommands.CS_runSetpoint(scorer, Degrees.of(20)));
        operatorController.b().onTrue(ScorerCommands.CS_runSetpoint(scorer, Degrees.of(210)));
        operatorController.axisGreaterThan(2, 0.1).onTrue(ScorerCommands.CS_setRunning(scorer, () -> 0.5)).onFalse(ScorerCommands.CS_setRunning(scorer, () -> 0.4));
        operatorController.axisGreaterThan(3, 0.1).onTrue(ScorerCommands.CS_setRunning(scorer, () -> -0.3
        )).onFalse(ScorerCommands.CS_setRunning(scorer, () -> 0.0));

        operatorController.y().onTrue(ScorerCommands.CS_intakeCoral(scorer));
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    //turns the tree on and off using the boolean that checks for the infinite loop
    public void switchTree() {
        if (blackboard.getBoolean("treeOn") == true) {
                blackboard.set("treeOn", false); 
        } else {
                blackboard.set("treeOn", true);
        }
}
    public void switchIntakeStateSim() {
        if (!blackboard.getBoolean("hasCoral") && !blackboard.getBoolean("hasAlgae")) {
                blackboard.set("hasCoral", true);
                blackboard.set("hasAlgae", true);
        } else {
                blackboard.set("hasCoral", false);
                blackboard.set("hasAlgae", false);
        Logger.recordOutput("BehaviorTree/hasCoral", blackboard.getBoolean("hasCoral"));
        }

        }


    public void getTreeTarget() {
        Targets targetValue = targetChooser.get();
        if (targetValue != null) {
        blackboard.set("target", targetValue);
        } else {
                blackboard.set("hasCoral", false);
                blackboard.set("hasAlgae", false);
        }
        }

    //Adds item for the stack - testing for the control tree
    public void addToStack() {
        Targets targetValue = targetChooser.get();
        stack.add(targetValue);
        blackboard.set("playerStation", playerStationChooser.get());
    }

   

    public void setTreeTarget() {
    blackboard.set("target", targetChooser.get());
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


    public void displayComponentPosesToAdvantageScope() {
    
        if (Constants.currentMode != Constants.Mode.SIM)
                return;

        Distance EL_simPosition = elevator.getELPosition();

        Transform3d elevator3d = new Transform3d(Inches.zero(), Inches.zero(), EL_simPosition,
                        new Rotation3d(0, 0, 0)); // 3d view changed to be a straight line up.
        Transform3d elevatorHalf3d = new Transform3d(Inches.zero(), Inches.zero(), EL_simPosition.div(2),
                        new Rotation3d(0, 0, 0));
        Rotation3d intakeRotation3d = new Rotation3d(intake.IN_getAngle().in(Radians), 0, 0);

        Rotation3d scorerRotation3d = new Rotation3d(scorer.CS_getAngle().in(Radians), 0, 0);

        Logger.recordOutput("Odometry/RobotComponentPoses", new Pose3d[] {
                        new Pose3d(Constants.scorerPoseOffset.getX() + elevator3d.getX(),
                                        Constants.scorerPoseOffset.getY() + elevator3d.getY(),
                                        Constants.scorerPoseOffset.getZ() + elevator3d.getZ(),
                                        Constants.scorerPoseOffset.getRotation().rotateBy(scorerRotation3d)),
                        new Pose3d(Constants.scorerRollerPoseOffset.getX() + elevator3d.getX(),
                                        Constants.scorerRollerPoseOffset.getY() + elevator3d.getY(),
                                        Constants.scorerRollerPoseOffset.getZ() + elevator3d.getZ(),
                                        Constants.scorerRollerPoseOffset.getRotation()
                                                        .rotateBy(scorerRotation3d)),
                        Constants.climberPoseOffset,
                        new Pose3d(Constants.intakePoseOffset.getX(), Constants.intakePoseOffset.getY(),
                                        Constants.intakePoseOffset.getZ(),
                                        Constants.intakeRollersPoseOffset.getRotation()
                                                        .rotateBy(intakeRotation3d)),
                        new Pose3d(Constants.intakeRollersPoseOffset.getX(),
                                        Constants.intakeRollersPoseOffset.getY(),
                                        Constants.intakeRollersPoseOffset.getZ(),
                                        Constants.intakeRollersPoseOffset.getRotation()
                                                        .rotateBy(intakeRotation3d)),
                        Constants.elevatorOneIntakeOffset.plus(elevator3d),
                        Constants.elevatorTwoIntakeOffset.plus(elevatorHalf3d),

        });
        }
}

        


