package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    private CTREConfigs() {
        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.ANGLE_CURRENT_LIMIT_ENABLED,
                Constants.SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT,
                Constants.SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT,
                Constants.SwerveConstants.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConstants.ANGLE_KP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConstants.ANGLE_KI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConstants.ANGLE_KD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConstants.ANGLE_KF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.DRIVE_CURRENT_LIMIT_ENABLED,
                Constants.SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT,
                Constants.SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT,
                Constants.SwerveConstants.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConstants.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConstants.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConstants.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConstants.DRIVE_KF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
    }
}