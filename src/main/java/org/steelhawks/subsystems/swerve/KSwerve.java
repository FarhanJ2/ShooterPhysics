package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.steelhawks.lib.COTSTalonFXSwerveConstants;
import org.steelhawks.lib.SwerveModuleConstants;

/** All constants for Swerve */
public final class KSwerve {

    public static final int PIGEON_ID = 0; // set up for your robot
    public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = // configure with your own robot drivetrain
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

    /* Robot Drivetrain Measurements */
    public static final double TRACK_WIDTH = Units.inchesToMeters(26.75); // configure on your own robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26.75); // configure on your own robot
    public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference; // configure on your own robot

    /* Swerve Drive Kinematics */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Swerve Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio; // configure on your own robot
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio; // configure on your own robot

    /* Motor Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    public static final double SLOW_MODE_MULTIPLIER = .2;

    /* Swerve Drive Current Limiting */
    /* Set this on your own robot */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;
    /* Angle Motor PID Values */
    public static final double angleKP = CHOSEN_MODULE.angleKP;
    public static final double angleKI = CHOSEN_MODULE.angleKI;
    public static final double angleKD = CHOSEN_MODULE.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.078838;
    public static final double driveKV = 2.5819;
    public static final double driveKA = 0.23783;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 4.5;
    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 10.0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    public static final double autoAlignKP = 0.07;
    public static final double autoAlignKI = 0;
    public static final double autoAlignKD = 0;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-130.17);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 6;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-51.34);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-63.90);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 11;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-175.96);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}
