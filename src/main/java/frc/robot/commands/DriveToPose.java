// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private static LoggedTunableNumber t_kP = new LoggedTunableNumber("Align/kP", AlignConstants.REEF_kP);
    private static LoggedTunableNumber t_kI = new LoggedTunableNumber("Align/kI", AlignConstants.REEF_kI);
    private static LoggedTunableNumber t_kD = new LoggedTunableNumber("Align/kD", AlignConstants.REEF_kD);
    private static LoggedTunableNumber t_MaxV = new LoggedTunableNumber("Align/Max Vel", 4.73);
    private static LoggedTunableNumber t_MaxA = new LoggedTunableNumber("Align/Max Acc", 20);

    private static LoggedTunableNumber t_RotkP = new LoggedTunableNumber("Align/Rot kP", AlignConstants.ROT_REEF_kP);
    private static LoggedTunableNumber t_RotkI = new LoggedTunableNumber("Align/Rot kI", AlignConstants.ROT_REEF_kI);
    private static LoggedTunableNumber t_RotkD = new LoggedTunableNumber("Align/Rot kD", AlignConstants.ROT_REEF_kD);
    private static LoggedTunableNumber t_RotMaxV = new LoggedTunableNumber("Align/Rot Max Vel", 8);
    private static LoggedTunableNumber t_RotMaxA = new LoggedTunableNumber("Align/Rot Max Acc", 20);

    private final ProfiledPIDController translationController = new ProfiledPIDController(
            AlignConstants.REEF_kP,
            AlignConstants.REEF_kI,
            AlignConstants.REEF_kD,
            new TrapezoidProfile.Constraints(8, 20));
    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            AlignConstants.ROT_REEF_kP,
            AlignConstants.ROT_REEF_kI,
            AlignConstants.ROT_REEF_kD,
            new TrapezoidProfile.Constraints(8, 20));

    private Drive drive;
    private Supplier<Pose2d> targetSupplier;
    private Supplier<Pose2d> robotSupplier;

    /** Creates a new DriveToPose. */
    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this.drive = drive;
        this.targetSupplier = target;
        this.robotSupplier = robot;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        translationController.reset(0);
        rotationController.reset(robotSupplier.get().getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updatePID();

        Pose2d targetPose = targetSupplier.get();
        Pose2d currentPose = robotSupplier.get();

        Translation2d translationError = targetPose.minus(currentPose).getTranslation();
        Rotation2d directionToTarget = translationError.getAngle();

        double translationOutput = translationController.calculate(translationError.getNorm(), 0);
        double rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Translation2d translationVelocity = new Translation2d(-translationOutput, directionToTarget);

        drive.runVelocity(new ChassisSpeeds(translationVelocity.getX(), translationVelocity.getY(), rotationOutput));

        Logger.recordOutput("DriveToPose/Target", targetPose);
        Logger.recordOutput("DriveToPose/Translation Output", translationOutput);
        Logger.recordOutput("DriveToPose/Rotation Output", rotationOutput);
        Logger.recordOutput("DriveToPose/Translation Error", translationError.getNorm());
        Logger.recordOutput(
                "DriveToPose/Rotation Error",
                targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        Logger.recordOutput("DriveToPose/Translation Velocity", translationVelocity);
        Logger.recordOutput("DriveToPose/Direction to Target", directionToTarget);
        Logger.recordOutput(
                "DriveToPose/Future",
                currentPose.transformBy(new Transform2d(
                        new Translation2d(translationVelocity.getX() * 0.1, translationVelocity.getY() * 0.1),
                        Rotation2d.fromRadians(rotationOutput * 0.1))));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void updatePID() {
        if (t_kP.hasChanged(hashCode()) || t_kI.hasChanged(hashCode()) || t_kD.hasChanged(hashCode())) {
            translationController.setPID(t_kP.get(), t_kI.get(), t_kD.get());
        }
        if (t_RotkP.hasChanged(hashCode()) || t_RotkI.hasChanged(hashCode()) || t_RotkD.hasChanged(hashCode())) {
            rotationController.setPID(t_RotkP.get(), t_RotkI.get(), t_RotkD.get());
        }
        if (t_MaxV.hasChanged(hashCode()) || t_MaxA.hasChanged(hashCode())) {
            translationController.setConstraints(new TrapezoidProfile.Constraints(t_MaxV.get(), t_MaxA.get()));
        }
        if (t_RotMaxV.hasChanged(hashCode()) || t_RotMaxA.hasChanged(hashCode())) {
            rotationController.setConstraints(new TrapezoidProfile.Constraints(t_RotMaxV.get(), t_RotMaxA.get()));
        }
    }
}
