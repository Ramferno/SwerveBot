package frc.lib.math;

public class Conversions {
    /**
     * @param counts    NEO position counts.
     * @param gearRatio Gear ratio between NEO and mechanism.
     * @return Mechanism degrees of rotation.
     */
    public static double neoToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees   Mechanism degrees of rotation.
     * @param gearRatio Gear ratio between NEO and mechanism.
     * @return NEO position counts.
     */
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param counts    NEO velocity counts.
     * @param gearRatio Gear ratio between NEO and mechanism (set to 1 for
     *                  NEO rpm).
     * @return Mechanism rpm.
     */
    public static double neoToRpm(double counts, double gearRatio) {
        double motorRpm = counts * (600.0 / 2048.0);
        double mechRpm = motorRpm / gearRatio;
        return mechRpm;
    }

    /**
     * @param rpm       Mechanism rpm.
     * @param gearRatio Gear ratio between NEO and mechanism (set to 1 for
     *                  NEO rpm).
     * @return NEO velocity counts.
     */
    public static double rpmToNeo(double rpm, double gearRatio) {
        double motorRPM = rpm * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param counts        NEO velocity counts.
     * @param maxVelocity   Maximum velocity
     * @return Velocity in m/s
     */
    public static double neoToMps(double counts, double maxVelocity) {
        return counts * maxVelocity;
    }

    /**
     * @param velocity      Velocity m/s.
     * @param maxVelocity   Maximum velocity
     * @return NEO velocity counts.
     */
    public static double mpsToNeo(double velocity, double maxVelocity) {
        return velocity / maxVelocity;
    }

    /**
     * @param positionCounts NEO position counts.
     * @param circumference  Wheel circumference.
     * @param gearRatio      Gear ratio between NEO and wheel.
     * @return Meters traveled.
     */
    public static double neoToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / gearRatio);
    }

    /**
     * @param meters        Meters traveled.
     * @param circumference Wheel circumference.
     * @param gearRatio     Gear ratio between NEO and wheel.
     * @return NEO position counts.
     */
    public static double metersToNeo(double meters, double circumference, double gearRatio) {
        return meters / (circumference / gearRatio);
    }
}
