package frc.team5115;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    private static final boolean isReplay = false;
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean SINGLE_MODE = false;
    public static final boolean DISABLE_AUTOMATION = false;
    public static final boolean ENABLE_DRIVER_CAMERA = true;
    public static final double RUMBLE_STRENGTH = 0.5;

    public static final double TELEOP_BARF_BURP_TIME = 0.5;
    public static final double AUTO_BARF_BURP_TIME = 0.5;

    public static final byte INTAKE_MOTOR_ID = -1;
    public static final byte INDEXER_MOTOR_ID = -1;
    public static final byte ARM_MOTOR_ID = -1;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static final double INTAKE_SPEED = 1.0;
    public static final double INTAKE_VOMIT_SPEED = -1.0;

    public static final double INDEX_SPEED = 0.5;
    public static final double INDEX_REJECT_SPEED = -0.15;
    public static final double INDEX_VOMIT_SPEED = -0.25;

    public static class SwerveConstants {
        public static final byte FRONT_LEFT_DRIVE_ID = -1;
        public static final byte FRONT_RIGHT_DRIVE_ID = -1;
        public static final byte BACK_LEFT_DRIVE_ID = -1;
        public static final byte BACK_RIGHT_DRIVE_ID = -1;

        public static final byte FRONT_LEFT_TURN_ID = -1;
        public static final byte FRONT_RIGHT_TURN_ID = -1;
        public static final byte BACK_LEFT_TURN_ID = -1;
        public static final byte BACK_RIGHT_TURN_ID = -1;

        private static RobotConfig ROBOT_CONFIG = null;

        public static RobotConfig getRobotConfig() {
            if (ROBOT_CONFIG == null) {
                try {
                    ROBOT_CONFIG = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return ROBOT_CONFIG;
        }

        public static final double MAX_LINEAR_SPEED = 5.0; // meters per second
        public static final double MAX_LINEAR_ACCEL = 9.81; // m/s^2, used for driver velocity slewing
        public static final double MAX_AUTOALIGN_LINEAR_SPEED = 4.0; // m/s

        // ! Reminder that X is front-back, Y is left-right
        private static final Distance FRAME_WIDTH_X = Inches.of(27 + 1/16);
        private static final Distance FRAME_WIDTH_Y = Inches.of(26 + 11/16);
        private static final Distance BUMPER_DEPTH = Inches.of(3);

        public static final Distance BUMPER_WIDTH_X = FRAME_WIDTH_X.plus(BUMPER_DEPTH.times(2));
        public static final Distance BUMPER_WIDTH_Y = FRAME_WIDTH_Y.plus(BUMPER_DEPTH.times(2));

        // Dimensions from CAD
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(22.125);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.5);

        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // Required for inverse kinematics. +x is forward, +y is left
        // The module order, as with everywhere else, is FL, FR, BL, BR
        public static final Translation2d[] MODULE_TRANSLATIONS =
                new Translation2d[] {
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / -2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / -2.0)
                };

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static class DriveMotorCurrentLimit {
            public static final int autoLimit = 60; // amps
            public static final int maxLimit = 50; // amps
            public static final int minLimit = 30; // amps
            public static final int stepSize = 5; // amps
            public static final double debounceTime = 5; // seconds
        }

        public static final int TurningMotorCurrentLimit = 20; // amps

        // TODO tune drive pids
        // TODO determine angularPID tolerance
        public static final double ANGULAR_PID_TOLERANCE = Math.toRadians(4);
        public static final PIDConstants ANGULAR_PID_CONSTANTS =
                new PIDConstants(
                        0.5 * SwerveConstants.MAX_ANGULAR_SPEED,
                        0.0 * SwerveConstants.MAX_ANGULAR_SPEED,
                        0.0 * SwerveConstants.MAX_ANGULAR_SPEED);

        public static final double LINEAR_PID_TOLERANCE = 0.04;
        public static final PIDConstants LINEAR_PID_CONSTANTS =
                new PIDConstants(
                        2.0 * SwerveConstants.MAX_AUTOALIGN_LINEAR_SPEED,
                        0.0 * SwerveConstants.MAX_AUTOALIGN_LINEAR_SPEED,
                        0.0 * SwerveConstants.MAX_AUTOALIGN_LINEAR_SPEED,
                        SwerveConstants.MAX_AUTOALIGN_LINEAR_SPEED * 0.5);
    }

    public static class AutoConstants {
        public static final Mass ROBOT_MASS = Pounds.of(83); // TODO update weight estimation

        public static boolean isInZone(Pose2d robot, Rectangle2d zone) {
            return zone.contains(robot.getTranslation());
        }
    }

    public static class VisionConstants {

        private static AprilTagFieldLayout loadCustomFieldLayout() {
            try {
                return new AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().getAbsolutePath()
                                + "/apriltag-fields/custom-2026-rebuilt-andymark.json");
            } catch (java.io.IOException e) {
                e.printStackTrace();
                return loadFullField();
            }
        }

        private static AprilTagFieldLayout loadFullField() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        }

        public static final AprilTagFieldLayout FIELD_LAYOUT = loadCustomFieldLayout();

        // * Camera sim values
        // TODO: Update camera values
        public static final int WIDTH_PX = 1280;
        public static final int HEIGHT_PX = 720;
        public static final double DIAG_FOV_DEGREES = 90;
        public static final double AVG_ERR_PX = 0.9;
        public static final double STD_DEV_ERR_PX = 0.3;
        public static final double FPS = 30;
        public static final double AVG_LATENCY_MS = 30;
        public static final double STD_DEV_LATENCY_MS = 10;

        // * Pose filtering values
        public static final double distanceThreshold = 2.0; // meters
        public static final double angleThreshold = 50.0; // degrees
        public static final double tagYawThreshold = 4.0; // degrees
        public static final double zTranslationThreshold = 0.15; // meters
        public static final double ambiguityThreshold = 0.5;
        // every tag beyond seeing two tags gives us an extra 30% of our threshold
        public static final double multiTagDistanceFactor = 0.3;

        // * Pose Estimation standard deviations *
        // Decreasing these increases trust in vision measurements
        private static final double VISION_STD_DEV_METERS = 0.1;
        private static final double VISION_STD_DEV_RADS = Units.degreesToRadians(20);

        // Decreasing these increases trust in the current state of the estimator
        private static final double STATE_STD_DEV_METERS = 0.1;
        private static final double STATE_STD_DEV_RADS = Units.degreesToRadians(10);

        public static final Matrix<N3, N1> VISION_STD_DEV =
                VecBuilder.fill(VISION_STD_DEV_METERS, VISION_STD_DEV_METERS, VISION_STD_DEV_RADS);
        public static final Matrix<N3, N1> STATE_STD_DEV =
                VecBuilder.fill(STATE_STD_DEV_METERS, STATE_STD_DEV_METERS, STATE_STD_DEV_RADS);

        /*
        The coordinate system for the camera to robot transforms is somewhat confusing.
        All lengths are in meters, and angles are in degrees.
        Positions are relative to the center of the robot.
        Positive X means that the camera is towards the front of the robot.
        Positive Y is directed to the left of the robot.
        Positive yaw points to the left, i.e. 90 degrees in yaw is directly pointed left.
        Positive pitch is actually pointed down, which is VERY important to remember.
        Positive roll is the camera rolling towards the robot's right side.
        */
        // TODO new measurements this year
        private static final double camX = Units.inchesToMeters(-1);
        private static final double camZ = Units.inchesToMeters(-1);

        private static final double leftY = Units.inchesToMeters(-1);
        private static final double rightY = Units.inchesToMeters(-1);

        private static final Angle roll = Degrees.of(-1);
        private static final Angle pitch = Degrees.of(-1);

        private static final Angle leftYaw = Degrees.of(-1);
        private static final Angle rightYaw = Degrees.of(-1);

        public static final Transform3d LEFT_CAM_TO_ROBOT =
                new Transform3d(camX, leftY, camZ, new Rotation3d(roll, pitch, leftYaw));
        public static final Transform3d RIGHT_CAM_TO_ROBOT =
                new Transform3d(camX, rightY, camZ, new Rotation3d(roll, pitch, rightYaw));
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    }
}
