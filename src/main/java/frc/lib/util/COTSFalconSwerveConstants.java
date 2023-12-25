package frc.lib.util;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSFalconSwerveConstants {
    public final double WHEEL_DIAMETER;
    public final double WHEEL_CIRCUMFERENCE;
    public final double ANGLE_GEAR_RATIO;
    public final double DRIVE_GEAR_RATIO;
    public final double ANGLE_KP;
    public final double ANGLE_KI;
    public final double ANGLE_KD;
    public final double ANGLE_KF;
    public final boolean DRIVE_MOTOR_INVERTED;
    public final boolean ANGLE_MOTOR_INVERTED;

    public COTSFalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP,
            double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert) {
        WHEEL_DIAMETER = wheelDiameter;
        WHEEL_CIRCUMFERENCE = wheelDiameter * Math.PI;
        ANGLE_GEAR_RATIO = angleGearRatio;
        DRIVE_GEAR_RATIO = driveGearRatio;
        ANGLE_KP = angleKP;
        ANGLE_KI = angleKI;
        ANGLE_KD = angleKD;
        ANGLE_KF = angleKF;
        DRIVE_MOTOR_INVERTED = driveMotorInvert;
        ANGLE_MOTOR_INVERTED = angleMotorInvert;
    }

    /** Swerve Drive Specialties - MK3 Module */
    public static COTSFalconSwerveConstants SDSMK3(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.9);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert);
    }

    /** Swerve Drive Specialties - MK4 Module */
    public static COTSFalconSwerveConstants SDSMK4(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert);
    }

    /** Swerve Drive Specialties - MK4i Module */
    public static COTSFalconSwerveConstants SDSMK4i(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.8285);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios {
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_STANDARD = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_FAST = (6.86 / 1.0);

        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75 / 1.0);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12 / 1.0);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14 / 1.0);

        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
    }
}