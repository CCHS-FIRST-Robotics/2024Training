import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        double driveVelocityMetersPerSec = 0.0;
        double turnVelocityRadPerSec = 0.0;

        double driveRelativePositionMeters = 0.0;
        double turnRelativePositionRad = 0.0;
        double turnAbsolutePositionRad = 0.0;

        double driveCurrentAmps = 0.0;
        double turnCurrentAmps = 0.0;
        double driveAppliedVolts = 0.0;
        double turnAppliedVolts = 0.0;
        double driveTempCelcius = 0.0;
        double turnTempCelcius = 0.0;
    }

    public default void updateInputs() {}

    public default void setDriveVoltage() {}
    public default void setTurnVoltage() {}
}
