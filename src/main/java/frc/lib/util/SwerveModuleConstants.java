package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;
    public final Rotation2d ANGLE_OFFSET;
    public final String CANBUS;

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, Rotation2d angleOffset) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        ANGLE_OFFSET = angleOffset;
        CANBUS = "";
    }

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     * @param angleOffset
     * @param canbus
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, Rotation2d angleOffset,
            String canbus) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        ANGLE_OFFSET = angleOffset;
        CANBUS = canbus;
    }

}