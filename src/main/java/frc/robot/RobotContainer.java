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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AlgaeHandler;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Arm arm;
    private final EndEffector endEffector;

    public final RobotVisualizer visualizer;
    public final AutoAlign align;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    public static final CommandXboxController controller = new CommandXboxController(0);
    public static final CommandXboxController opController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                align = new AutoAlign(drive::getPose);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive // ,
                        // new VisionIOPhotonVisionSim(
                        //         camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        // new VisionIOPhotonVisionSim(
                        //         camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                        );
                arm = new Arm(new ArmIOSim());
                endEffector = new EndEffector(new EndEffectorIOSim(
                        driveSimulation, () -> arm.getArmPosition().getWristTransform()));
                align = new AutoAlign(driveSimulation::getSimulatedDriveTrainPose);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                arm = new Arm(new ArmIO() {});
                endEffector = new EndEffector(new EndEffectorIO() {});
                align = new AutoAlign(drive::getPose);

                break;
        }

        // Set up auto routines
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

        visualizer = new RobotVisualizer(arm::getArmPosition);

        // Configure the button bindings
        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                true));

        endEffector.setDefaultCommand(new RunCommand(() -> endEffector.intake(), endEffector));

        // Controls can be found here
        // https://docs.google.com/spreadsheets/d/1LB6nTpDfxbCZiFzkx_2DcHvuDDgMP4XP9NIPqSqYTP4/

        // --- Driver Controls ---
        controller
                .povUp()
                .whileTrue(
                        new ConditionalCommand(align.stationAlign(drive), align.netAlign(drive), endEffector::isCoral));
        controller.povLeft().whileTrue(align.reefAlignLeft(drive));
        controller.povDown().whileTrue(align.reefAlignMid(drive));
        controller.povRight().whileTrue(align.reefAlignRight(drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        controller
                .leftBumper()
                .and(() -> !endEffector.hasAlgae() && !endEffector.hasCoral())
                .onTrue(arm.followStateSupplier(() -> ArmState.groundIntake(endEffector.isCoral())))
                .onFalse(arm.followStateSupplier(() -> ArmState.hold(endEffector.isCoral())));
        controller.rightBumper().whileTrue(new RunCommand(() -> endEffector.outtake(align.isForwards()), endEffector));

        // --- Operator Controls ---
        opController.leftBumper().onTrue(new InstantCommand(() -> endEffector.setCoral(true)));
        opController.rightBumper().onTrue(new InstantCommand(() -> endEffector.setCoral(false)));

        opController.start().onTrue(new InstantCommand(() -> endEffector.setHorizontal(true)));
        opController.back().onTrue(new InstantCommand(() -> endEffector.setHorizontal(false)));

        opController.povLeft().onTrue(arm.applyState(ArmState.TUNABLE));
        opController.povRight().onTrue(arm.applyState(ArmState.CORAL_STATION));

        opController
                .a()
                .onTrue(arm.followStateSupplier(() -> ArmState.of(1, endEffector.isCoral(), align.isForwards())));
        opController
                .x()
                .onTrue(arm.followStateSupplier(() -> ArmState.of(2, endEffector.isCoral(), align.isForwards())));
        opController
                .b()
                .onTrue(arm.followStateSupplier(() -> ArmState.of(3, endEffector.isCoral(), align.isForwards())));
        opController
                .y()
                .onTrue(arm.followStateSupplier(() -> ArmState.of(4, endEffector.isCoral(), align.isForwards())));

        // new Trigger(() -> align.isAlignedDebounced() && endEffector.hasCoral())
        //         .onTrue(new InstantCommand(() -> endEffector.outtake()));

        if (Constants.currentMode == Constants.Mode.SIM) {
            new Trigger(DriverStation::isAutonomousEnabled)
                    .onTrue(new WaitCommand(15.3)
                            .andThen(new InstantCommand(() -> DriverStationSim.setEnabled(false)))
                            .andThen(new WaitCommand(3).andThen(new InstantCommand(() -> {
                                DriverStationSim.setAutonomous(false);
                                DriverStationSim.setEnabled(true);
                            })))
                            .ignoringDisable(true));
            new Trigger(DriverStation::isTeleopEnabled)
                    .whileTrue(new WaitCommand(135.3)
                            .andThen(new InstantCommand(() -> DriverStationSim.setEnabled(false))));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.setPose(new Pose2d(12, 2, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
        AlgaeHandler.getInstance().reset();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "FieldSimulation/Staged Algae", AlgaeHandler.getInstance().periodic());
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
