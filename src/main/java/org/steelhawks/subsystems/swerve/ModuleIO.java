package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);
    void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);
    void resetToAbsolute();
    SwerveModuleState getState();
    SwerveModulePosition getPosition();
    /** Returns 0 volts if it is a simulation, returns drive and angle motor voltage usage if real. */
    double getDriveVoltage();

    /** For logging on AdvantageKit */
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRads;
        public double driveVelocityRadsPerSec;
        public Rotation2d anglePosition;
        public double angleVelocityRadsPerSec;
        public SwerveModuleState desiredState;
        public boolean isOpenLoop;
        public double driveAppliedVoltage;
    }

    default void updateInputs(ModuleIOInputs inputs) {}
}
