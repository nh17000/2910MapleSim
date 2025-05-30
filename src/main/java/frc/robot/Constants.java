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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Arrays;
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

    public static final class ArmConstants {
        public enum ArmState {
            STOWED(0, 0, 125),
            L4(68, 39.75, 45),
            L3(56, 19, 95),
            L2(38, 9.45, 118),
            L1(35, 0, -20),
            L4_BACKWARDS(100, 40.5, 149.5),
            L3_BACKWARDS(100, 11, 115),
            L2_BACKWARDS(107, 0, 124),
            CORAL_STATION(67, 5.6, -31),
            GROUND_CORAL_INTAKE(0, 0, 0),
            NET(90, 40.5, -20);

            public final ArmPosition position;

            private ArmState(double pivotDegrees, double extensionInches, double wristDegrees) {
                this.position = new ArmPosition(
                        Units.degreesToRadians(pivotDegrees),
                        Units.inchesToMeters(extensionInches),
                        Units.degreesToRadians(wristDegrees));
            }
        }

        public static final TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 25;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 25;

            config.MotionMagic.MotionMagicCruiseVelocity = 1000;
            config.MotionMagic.MotionMagicAcceleration = 2000;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.25;
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

            config.MotionMagic.MotionMagicCruiseVelocity = 1000;
            config.MotionMagic.MotionMagicAcceleration = 2000;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.25;
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

            config.MotionMagic.MotionMagicCruiseVelocity = 1000;
            config.MotionMagic.MotionMagicAcceleration = 2000;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.3;
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

        public static final double PIVOT_GEAR_RATIO = 725.0 / 6.0; // 120.83
        public static final double EXTENSION_GEAR_RATIO = 6.28;
        public static final double WRIST_GEAR_RATIO = 30; // ?

        public static final double ARM_MASS_KG = Units.lbsToKilograms(20);
        public static final double ARM_SHOULDER_TO_WRIST_LENGTH = VisualizerConstants.WRIST_OFFSET.getNorm();

        public static final double PIVOT_MIN_ANGLE = 0;
        public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(120);

        public static final double EXTENSION_DRUM_RADIUS = Units.inchesToMeters(2.005 / 2); // ?
        public static final double EXTENSION_MIN_LENGTH = 0;
        public static final double EXTENSION_MAX_LENGTH = Units.inchesToMeters(40.5);

        public static final double WRIST_MASS_KG = Units.lbsToKilograms(5);
        public static final double WRIST_LENGTH = Units.inchesToMeters(6);
        public static final double WRIST_STARTING_ANGLE = Units.degreesToRadians(125);
        public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-35);
        public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(150);
    }

    public static final class EndEffectorConstants {
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
    }

    public static final class AlignConstants {
        public static final double ALIGN_KS = 0.09; // 0.009

        // tx and ty tolerances with setpoint
        public static final double ALIGN_TOLERANCE_PIXELS = 0.5;
        // don't try translationally aligning unless rotation is already aligned within this tolerance
        public static final double ALIGN_ROT_TOLERANCE_DEGREES = 5;

        // reduce speed by 1/4 every tick when an april tag is not seen
        public static final double ALIGN_DAMPING_FACTOR = 0.75;
        public static final double ALIGN_SPEED_DEADBAND = 0.025;

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0); // 12.94 //12.97

        // target relative
        public static final double REEF_ALIGN_MID_TX = 0; // 0.28575
        public static final double REEF_ALIGN_LEFT_TX = -BRANCH_SPACING; // - 0.05 + 0.01;
        public static final double REEF_ALIGN_RIGHT_TX = BRANCH_SPACING; // - 0.03 + 0.01;
        public static final double REEF_ALIGN_TZ = 0.5414; // target relative

        public static final double STATION_ALIGN_TX = 0.07;
        public static final double STATION_ALIGN_TZ = 0;

        public static final double REEF_kP = 0.85; // Tune all PID values
        public static final double REEF_kI = 0;
        public static final double REEF_kD = 0;

        public static final double REEF_Forward_kP = 0.75; // Tune all PID values

        public static final double ROT_REEF_kP = 0.02; // Tune all PID values
        public static final double ROT_REEF_kI = 0;
        public static final double ROT_REEF_kD = 0;
        public static final double ROT_KS = ALIGN_KS;
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);

        public static final Translation2d BLUE_NPS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(291.176));
        public static final Translation2d BLUE_PS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824));

        public static final Translation2d RED_NPS_CORAL_STATION =
                new Translation2d(FIELD_LENGTH - Units.inchesToMeters(33.526), Units.inchesToMeters(291.176));
        public static final Translation2d RED_PS_CORAL_STATION =
                new Translation2d(FIELD_LENGTH - Units.inchesToMeters(33.526), Units.inchesToMeters(25.824));

        public static final List<Pose2d> CORAL_STATIONS = Arrays.asList(
                new Pose2d(BLUE_NPS_CORAL_STATION, new Rotation2d(125)),
                new Pose2d(BLUE_PS_CORAL_STATION, new Rotation2d(-125)),
                new Pose2d(RED_NPS_CORAL_STATION, new Rotation2d(-125)),
                new Pose2d(RED_PS_CORAL_STATION, new Rotation2d(125)));

        // the top of the branch (L4) is ~2" behind the april tag
        public static final double BRANCH_OFFSET_BEHIND_APRILTAG = Units.inchesToMeters(2.049849);
        public static final double L4_HEIGHT = Units.inchesToMeters(72);

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
