// Copyright 2021-2025 FRC 6328
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

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.dyn4j.world.Island;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.controller.PIDController;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 8.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    private static final double LINEAR_KP = 10.0; //0.1
    private static final double LINEAR_KD = 1.0; //0.01;
    private static final double LINEAR_MAX_ACCELERATION = 26.62;
    private static final double LINEAR_xKP = 0.05;
    private static final double LINEAR_xKD = 0.005;
    private static final double LINEAR_yKP = 0.5;
    private static final double LINEAR_yKD = 0.05;

    private static final LinearFilter filter = LinearFilter.singlePoleIIR(0.01, 0.02);

    private final Pose2d lineUpPose = new Pose2d(5.8, 4, new Rotation2d(3.1415926589793));

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }


// Prevent the path from being flipped if the coordinates are already correct

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command

                    //add an if statement here and change the chassis speeds to move towards the pose2d when the button is being pressed
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
                },
                drive);
    }

    public static Command robotCentricJoystickDrive(
        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
                // Get linear velocity
                Translation2d linearVelocity =
                        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Convert to field relative speeds & send command

                //add an if statement here and change the chassis speeds to move towards the pose2d when the button is being pressed
                ChassisSpeeds speeds = new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * 0.25,
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * 0.25,
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        new Rotation2d()));
            },
            drive);
}

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command allAxisAutoAlign(
        Drive drive, Vision vision, DoubleSupplier xOffset, DoubleSupplier yOffset, Supplier<Rotation2d> rotationSupplier, BooleanSupplier isLeftReef) {

        ProfiledPIDController xlinearController = new ProfiledPIDController(
                LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));
        ProfiledPIDController ylinearController = new ProfiledPIDController(
                LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {

                Logger.recordOutput("LineUp/CommandXCenterOffset", xOffset);
                Logger.recordOutput("LineUp/CommandYCenterOffset", yOffset);
                // Get linear velocity

                double xTranslation = xlinearController.calculate(
                        xOffset.getAsDouble(),
                        drive.isLeftReef(isLeftReef.getAsBoolean()));

                double yTranslation = ylinearController.calculate(
                        yOffset.getAsDouble(),
                        0.45);

                // Apply rotation deadbande
                double omega = angleController.calculate(
                        drive.getRotation().getRadians(),
                        rotationSupplier.get().getRadians());

                // Convert to field relative speeds & send command

                //add an if statement here and change the chassis speeds to move towards the pose2d when the button is being pressed
                ChassisSpeeds speeds = new ChassisSpeeds(
                        0.0,
                        xTranslation,
                        omega);
                drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        new Rotation2d()));
            },
            drive);
}


public static Command xAxisAutoAlign(
        Drive drive, DoubleSupplier xOffset, BooleanSupplier isLeftReef) {

    // Create PID controller

    ProfiledPIDController xlinearController = new ProfiledPIDController(
        LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

        ProfiledPIDController ylinearController = new ProfiledPIDController(
                LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));
    // Construct command
    return Commands.run(
                    () -> {
                        
                        double xTranslation;
                        if (isLeftReef.getAsBoolean()) {                                                                                        
                            xTranslation = xlinearController.calculate(
                                    xOffset.getAsDouble(),
                                    0.30);
                        } else {
                            xTranslation = xlinearController.calculate(
                                    xOffset.getAsDouble(),
                                    -0.12);
                        }
                        // Calculate angular speed

                        
                                                // Convert to field relative speeds & send command
                                                ChassisSpeeds speeds = new ChassisSpeeds(
                                                        0.0,
                                                        xTranslation,
                                0.0);
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                new Rotation2d()));
                    },
                    drive);

            
}

public static Command yAxisAutoAlign(
        Drive drive, DoubleSupplier yOffset, BooleanSupplier isLeftReef) {

    // Create PID controller


        ProfiledPIDController ylinearController = new ProfiledPIDController(
                LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));
    // Construct command
    return Commands.run(
                    () -> {
                        
                        double yTranslation = ylinearController.calculate(
                                yOffset.getAsDouble(),
                                0.65
                        );
                        // Calculate angular speed

                        
                                                // Convert to field relative speeds & send command
                                                ChassisSpeeds speeds = new ChassisSpeeds(
                                                        -yTranslation,
                                                        0.0,
                                0.0);
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                new Rotation2d()));
                    },
                    drive).until(() -> Math.abs(yOffset.getAsDouble()) < 0.67);
            
}

public static SequentialCommandGroup sequentialAutoAlign(Drive drive, DoubleSupplier xOffset, DoubleSupplier yOffset, BooleanSupplier isLeftReef) {
    return new SequentialCommandGroup(
        yAxisAutoAlign(drive, yOffset, isLeftReef),
        xAxisAutoAlign(drive, xOffset, isLeftReef)
    );
}

public static Command xyAxisAutoAlign(
        Drive drive, DoubleSupplier xOffset, DoubleSupplier yOffset, BooleanSupplier isLeftReef) {

    // Create PID controller

    ProfiledPIDController xlinearController = new ProfiledPIDController(
        LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

    ProfiledPIDController ylinearController = new ProfiledPIDController(
        LINEAR_KP, 0.0, LINEAR_KD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));
    // Construct command
    return Commands.run(
                    () -> {
                        
                        double xTranslation;
                        if (isLeftReef.getAsBoolean()) {                                                                                        
                            xTranslation = xlinearController.calculate(
                                    xOffset.getAsDouble(),
                                    0.30);
                        } else {
                            xTranslation = xlinearController.calculate(
                                    xOffset.getAsDouble(),
                                    -0.12);
                        }

                        double yTranslation = ylinearController.calculate(
                                yOffset.getAsDouble(),
                                0.65
                        );

                        filter.calculate(xTranslation);
                        filter.calculate(yTranslation);
                        xTranslation = MathUtil.applyDeadband(xTranslation, 0.15);
                        yTranslation = MathUtil.applyDeadband(xTranslation, 0.15);
                        Logger.recordOutput("LineUp/xMetersPerSecond", xTranslation);
                        Logger.recordOutput("LineUp/yMetersPerSecond", -yTranslation);

                        
                                                // Convert to field relative speeds & send command
                                                ChassisSpeeds speeds = new ChassisSpeeds(
                                                        -yTranslation,
                                                        xTranslation,
                                0.0);
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                new Rotation2d()));
                    },
                    drive);

            
}       public static Command xyAlign(
        Drive drive, DoubleSupplier tagSize, DoubleSupplier pitchAngle, BooleanSupplier isLeftReef) {

    // Create PID controller

    ProfiledPIDController xlinearController = new ProfiledPIDController(
        LINEAR_xKP, 0.0, LINEAR_xKD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

    ProfiledPIDController ylinearController = new ProfiledPIDController(
        LINEAR_yKP, 0.0, LINEAR_yKD, new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));
    // Construct command
    return Commands.run(
                    () -> {

                        double xTranslation;
                        double yTranslation;

                                      
                        if (isLeftReef.getAsBoolean()) {
                        xTranslation = xlinearController.calculate(
                                    pitchAngle.getAsDouble(),
                                    8);
                        yTranslation = ylinearController.calculate(
                                        tagSize.getAsDouble(),
                                        6);
                        } else {
                        xTranslation = xlinearController.calculate(
                                pitchAngle.getAsDouble(),
                                -13);
                        yTranslation = ylinearController.calculate(tagSize.getAsDouble(), 7);
                        
                        }

                        xTranslation = MathUtil.applyDeadband(xTranslation, 0.75);
                        yTranslation = MathUtil.applyDeadband(yTranslation, 0.75);


                        Logger.recordOutput("LineUp/xMetersPerSecond", xTranslation);
                        Logger.recordOutput("LineUp/yMetersPerSecond", yTranslation);

                        
                                                // Convert to field relative speeds & send command
                                                ChassisSpeeds speeds = new ChassisSpeeds(
                                                        yTranslation,
                                                        xTranslation,
                                0.0);
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                new Rotation2d()));
                    },
                    drive);

            
}




public static Command omegaAxisAutoAlign(
        Drive drive, Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
                    () -> {

                        // Calculate angular speed
                        double omega = angleController.calculate(
                                drive.getRotation().getRadians(),
                                rotationSupplier.get().getRadians());

                        // Convert to field relative speeds & send command
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                0.0,
                                0.0,
                                omega);
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                new Rotation2d()));
                    },
                    drive)

            // Reset PID controller when command starts
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
}

public static SequentialCommandGroup AutoAlign(Drive drive, Vision vision, DoubleSupplier xOffset, DoubleSupplier yOffset, Supplier<Rotation2d> rotationSupplier, BooleanSupplier isLeftReef) {
        return new SequentialCommandGroup(
                omegaAxisAutoAlign(drive, rotationSupplier).raceWith(new WaitCommand(0.25)),
                allAxisAutoAlign(drive, vision, xOffset, yOffset, rotationSupplier, isLeftReef)
        );
}




    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
