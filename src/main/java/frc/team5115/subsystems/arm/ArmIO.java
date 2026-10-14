package frc.team5115.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.util.InterfaceReplayCheck;

import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armAngle = new Rotation2d();
        public double armVelocityRPM = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    public default void setArmVoltage(double volts) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    public default ArrayList<SparkMax> getSparks() {
        InterfaceReplayCheck.warnOnNotReplay();
        return new ArrayList<>();
    }
}
