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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double LOOP_PERIOD = 0.02; // 20ms
    public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
    public static final double NOMINAL_VOLTAGE = 12;

    public static final class ArmConstants {
        public enum ArmState {
            // values obtained from CAD configurations in onshape
            STOWED(0, 0, 125),
            L4(68, 39.75, 45),
            L3(56, 19, 95),
            L2(38, 9.45, 118),
            L1(35, 0, -20),
            L4_BACKWARDS(100, 40.5, 149.5),
            L3_BACKWARDS(100, 11, 115),
            L2_BACKWARDS(107, 0, 124),
            CORAL_STATION(67, 5.6, -31),
            NET(90, 40.5, -20),
            // values obtained from heuristic estimation in sim
            GROUND_INTAKE(0, 0, 0),
            LOW_ALGAE(45, 3, -15),
            LOW_ALGAE_BACKWARDS(107, 0, 80),
            HIGH_ALGAE(63, 14, -30),
            HIGH_ALGAE_BACKWARDS(100, 16, 80),
            LOLLIPOP(0, 6, 42.5); // or procesor

            public final ArmPosition position;

            private ArmState(double pivotDegrees, double extensionInches, double wristDegrees) {
                this.position = new ArmPosition(
                        Units.degreesToRadians(pivotDegrees),
                        Units.inchesToMeters(extensionInches),
                        Units.degreesToRadians(wristDegrees));
            }

            public static ArmState of(int level, boolean isCoral, boolean isForwards) {
                if (isCoral) {
                    switch (level) {
                        case 4:
                            return isForwards ? ArmState.L4 : ArmState.L4_BACKWARDS;
                        case 3:
                            return isForwards ? ArmState.L3 : ArmState.L3_BACKWARDS;
                        case 2:
                            return isForwards ? ArmState.L2 : ArmState.L2_BACKWARDS;
                        default:
                            return ArmState.L1;
                    }
                } else {
                    switch (level) {
                        case 4:
                            return ArmState.NET;
                        case 3:
                            return isForwards ? ArmState.HIGH_ALGAE : ArmState.HIGH_ALGAE_BACKWARDS;
                        case 2:
                            return isForwards ? ArmState.LOW_ALGAE : ArmState.LOW_ALGAE_BACKWARDS;
                        default:
                            return ArmState.LOLLIPOP;
                    }
                }
            }
        }

        public static final TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 25;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 25;

            config.MotionMagic.MotionMagicCruiseVelocity = Arm.getPivotMotorRots(Units.degreesToRadians(1000));
            config.MotionMagic.MotionMagicAcceleration = Arm.getPivotMotorRots(Units.degreesToRadians(600));

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            return config;
        }

        public static final TalonFXConfiguration getExtensionConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 30;

            config.MotionMagic.MotionMagicCruiseVelocity = Arm.getExtensionMotorRots(Units.inchesToMeters(200));
            config.MotionMagic.MotionMagicAcceleration = Arm.getExtensionMotorRots(Units.inchesToMeters(400));

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.5;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            return config;
        }

        public static final TalonFXConfiguration getWristConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 30;

            config.MotionMagic.MotionMagicCruiseVelocity = Arm.getWristMotorRots(Units.degreesToRadians(2000));
            config.MotionMagic.MotionMagicAcceleration = Arm.getWristMotorRots(Units.degreesToRadians(4500));

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.15;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            return config;
        }

        public static final int PIVOT_ONE_ID = 20;
        public static final int PIVOT_TWO_ID = 21;
        public static final int PIVOT_THREE_ID = 22;

        public static final int EXTENSION_ONE_ID = 30;
        public static final int EXTENSION_TWO_ID = 31;
        public static final int EXTENSION_THREE_ID = 32;

        public static final int WRIST_ID = 40;

        public static final double PIVOT_GEAR_RATIO = (60. / 12.) * (60. / 16.) * (58. / 9.); // 120.83
        public static final double EXTENSION_GEAR_RATIO = (32. / 16.) * (40. / 26.) * (50. / 20.) * (62. / 76.); // 6.28
        public static final double WRIST_GEAR_RATIO = (50. / 9.) * (38. / 12.) * (38. / 12.); // 55.71

        public static final double ARM_MASS_KG = Units.lbsToKilograms(20);
        public static final double ARM_SHOULDER_TO_WRIST_LENGTH = VisualizerConstants.WRIST_OFFSET.getNorm();

        public static final double PIVOT_MIN_ANGLE = 0;
        public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(120);
        public static final DCMotor PIVOT_MOTORS = DCMotor.getKrakenX60(3);

        public static final double EXTENSION_DRUM_RADIUS = Units.inchesToMeters(0.25 * 16.0 / Math.PI * 0.5); // ~0.64"
        public static final double EXTENSION_MIN_LENGTH = 0;
        public static final double EXTENSION_MAX_LENGTH = Units.inchesToMeters(40.5);
        public static final DCMotor EXTENSION_MOTORS = DCMotor.getKrakenX60(3);

        public static final double WRIST_MASS_KG = Units.lbsToKilograms(5);
        public static final double WRIST_LENGTH = Units.inchesToMeters(6);
        public static final double WRIST_STARTING_ANGLE = Units.degreesToRadians(125);
        public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-35);
        public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(150);
        public static final DCMotor WRIST_MOTOR = PhoenixUtil.getKrakenX44(1);
    }

    public static final class EndEffectorConstants {
        public enum EEState {
            VERTICAL_CORAL_INTAKE(-0.3, -0.3),
            VERTICAL_CORAL_OUTTAKE_FWD(-0.5, -0.5),
            VERTICAL_CORAL_OUTTAKE_BWD(0.5, -0.5),
            HORIZONTAL_CORAL_INTAKE(-0.3, 0.3),
            HORIZONTAL_CORAL_OUTTAKE(0.5, 0.5),
            ALGAE_INTAKE(0.0, 0.7),
            ALGAE_OUTTAKE(0.0, -1.0),
            OFF(0.0, 0.0);

            public final double leftVolts;
            public final double rightVolts;
            public final double topVolts;

            private EEState(double leftVolts, double topVolts) {
                this.leftVolts = leftVolts;
                this.rightVolts = -leftVolts;
                this.topVolts = topVolts;
            }
        }

        public static final TalonFXConfiguration getLRConfigs() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 30;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final TalonFXConfiguration getTopConfigs() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 100;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 100;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int LEFT_ID = 41;
        public static final int RIGHT_ID = 42;
        public static final int TOP_ID = 43;

        public static final int CAN_RANGE_ID = 44;

        public static final double TRANSLATIONAL_TOLERANCE = Units.inchesToMeters(16);

        public static final double INTAKING_TIME = 0.5;
        public static final double DROP_COOLDOWN = 2.0;

        public static final double LR_GEAR_RATIO = 1.0 / ((40.0 / 12.0) * (40.0 / 14.0));
        public static final double LR_MASS = Units.lbsToKilograms(0.5);
        public static final double LR_RADIUS = Units.inchesToMeters(2);
        public static final double LR_MOI = 1.0 / 2.0 * LR_MASS * LR_RADIUS * LR_RADIUS;

        public static final DCMotor LR_MOTOR = PhoenixUtil.getKrakenX44(1);
    }

    public static final class AlignConstants {
        // public static final double ALIGN_KS = 0.1;

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0);

        // target relative
        public static final double REEF_ALIGN_MID_TX = 0.08;
        public static final double REEF_ALIGN_LEFT_TX = -BRANCH_SPACING; // - 0.05 + 0.01;
        public static final double REEF_ALIGN_RIGHT_TX = BRANCH_SPACING; // - 0.03 + 0.02;
        public static final double REEF_ALIGN_TZ = Units.inchesToMeters(22); // 18

        public static final double STATION_ALIGN_TX = 0.0;
        public static final double STATION_ALIGN_TZ = Units.inchesToMeters(18);

        public static final double REEF_kP = 5.0;
        public static final double REEF_kI = 0.0;
        public static final double REEF_kD = 0.0;

        public static final double ROT_REEF_kP = 5.0;
        public static final double ROT_REEF_kI = 0.0;
        public static final double ROT_REEF_kD = 0.0;

        public static final double ALIGN_ROT_TOLERANCE = Units.degreesToRadians(3);
        public static final double ALIGN_TRANSLATION_TOLERANCE = Units.inchesToMeters(2);
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
        public static final int[] ALL_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17, 7, 6, 11, 10, 9, 8};

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);

        public static final List<Pose2d> CORAL_STATIONS = new ArrayList<>();

        static {
            for (int tag : BLUE_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
            for (int tag : RED_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
        }

        public static final Pose3d[] REEF_TAG_POSES = new Pose3d[RED_REEF_TAG_IDS.length + BLUE_REEF_TAG_IDS.length];

        static {
            int i = 0;
            for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
            for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
        }

        public static final List<Pose2d> REEF_TAGS = new ArrayList<>();

        static {
            for (Pose3d tag : REEF_TAG_POSES) {
                REEF_TAGS.add(tag.toPose2d());
            }
        }

        public static final Transform3d HIGH_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(39.575), Rotation3d.kZero);
        public static final Transform3d LOW_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(23.675), Rotation3d.kZero);

        public static final Pose3d[] REEF_ALGAE_POSES = new Pose3d[REEF_TAG_POSES.length];

        static {
            for (int i = 0; i < REEF_ALGAE_POSES.length; i++) {
                REEF_ALGAE_POSES[i] = REEF_TAG_POSES[i].plus(i % 2 == 0 ? HIGH_ALGAE_TRANSFORM : LOW_ALGAE_TRANSFORM);
            }
        }

        public static final double BARGE_X = FIELD_LENGTH / 2.0;
        public static final double BARGE_WIDTH = Units.inchesToMeters(40) / 2.0;
        public static final double BARGE_HEIGHT = Units.inchesToMeters(74 + 8);
        public static final double BARGE_HEIGHT_TOLERANCE = Units.inchesToMeters(12);
    }

    public static final class VisualizerConstants {
        public static final Translation3d STAGE0_ZERO = new Translation3d(-0.26035, 0, 0.32385);
        public static final Translation3d WRIST_ZERO = new Translation3d(0.368313, 0, 0.196875);
        public static final Translation3d WRIST_OFFSET = WRIST_ZERO.minus(STAGE0_ZERO);

        public static final Transform3d CORAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(5.25), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(45), 0)),
                new Rotation3d(0, -Units.degreesToRadians(125), 0));

        public static final Transform3d HORIZONTAL_CORAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.5), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(96), 0)),
                new Rotation3d(0, 0, Units.degreesToRadians(90)));

        public static final Transform3d ALGAE_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(16), 0, 0).rotateBy(CORAL_TRANSFORM.getRotation()),
                Rotation3d.kZero);
    }
}
