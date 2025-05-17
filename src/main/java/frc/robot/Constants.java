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

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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
            NET(90, 40.5, -20);
            public final double pivotRads, extensionMeters, wristRads;

            private ArmState(double pivotDegrees, double extensionInches, double wristDegrees) {
                this.pivotRads = Units.degreesToRadians(pivotDegrees);
                this.extensionMeters = Units.inchesToMeters(extensionInches);
                this.wristRads = Units.degreesToRadians(wristDegrees);
            }
        }

        public static final int PIVOT_ONE_ID = 20;
        public static final int PIVOT_TWO_ID = 21;
        public static final int PIVOT_THREE_ID = 22;

        public static final int EXTENSION_ONE_ID = 30;
        public static final int EXTENSION_TWO_ID = 31;
        public static final int EXTENSION_THREE_ID = 32;

        public static final int WRIST_ID = 40;

        public static final int PIVOT_CURRENT_LIMIT = 25;
        public static final int EXTENSION_CURRENT_LIMIT = 30;
        public static final int WRIST_CURRENT_LIMIT = 30;

        // TODO: tune PIDFF
        public static final Slot0Configs PIVOT_SLOT0_CONFIGS = new Slot0Configs()
                .withKG(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.0);

        public static final Slot0Configs EXTENSION_SLOT0_CONFIGS = new Slot0Configs()
                .withKG(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.0);

        public static final Slot0Configs WRIST_SLOT0_CONFIGS = new Slot0Configs()
                .withKG(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.15)
                .withKI(0.0)
                .withKD(0.0);

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
