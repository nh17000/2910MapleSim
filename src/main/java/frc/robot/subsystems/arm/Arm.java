package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    @AutoLogOutput
    @Getter
    private ArmState state = ArmState.STOWED;

    private static final LoggedTunableNumber tunablePivot = new LoggedTunableNumber("Arm/Tunable Pivot", 0.0);
    private static final LoggedTunableNumber tunableExtension = new LoggedTunableNumber("Arm/Tunable Extension", 0.0);
    private static final LoggedTunableNumber tunableWrist = new LoggedTunableNumber("Arm/Tunable Wrist", 0.0);

    private final SysIdRoutine pivotSysID;

    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;

        pivotSysID = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.2), // Use default ramp rate (1 V/s)
                        Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
                        Second.of(5), // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> io.setWristVolts(volts.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        ArmPosition setpoint = state.position;
        if (state == ArmState.TUNABLE) {
            setpoint = new ArmPosition(
                    Units.degreesToRadians(tunablePivot.get()),
                    Units.inchesToMeters(tunableExtension.get()),
                    Units.degreesToRadians(tunableWrist.get()));
        }

        io.setPivotSetpoint(setpoint.getPivotRads() / ArmConstants.PIVOT_P_COEFFICIENT, getPivotkG());
        io.setExtensionSetpoint(setpoint.getExtensionMeters() / ArmConstants.EXTENSION_P_COEFFICIENT, getExtensionkG());
        io.setWristSetpoint(setpoint.getWristRads() / ArmConstants.WRIST_P_COEFFICIENT);

        Logger.recordOutput("Arm/Pivot Setpoint Rots", setpoint.getPivotRads() / ArmConstants.PIVOT_P_COEFFICIENT);
        Logger.recordOutput(
                "Arm/Extension Setpoint Rots", setpoint.getExtensionMeters() / ArmConstants.EXTENSION_P_COEFFICIENT);
        Logger.recordOutput("Arm/Wrist Setpoint Rots", setpoint.getWristRads() / ArmConstants.WRIST_P_COEFFICIENT);
        Logger.recordOutput("Arm/Component Setpoints", state.position.getComponentTransforms());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return pivotSysID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return pivotSysID.dynamic(direction);
    }

    public Command applyState(ArmState desiredState) {
        return runOnce(() -> this.state = desiredState);
    }

    public Command followStateSupplier(Supplier<ArmState> stateSupplier) {
        return run(() -> this.state = stateSupplier.get());
    }

    @AutoLogOutput
    public double getPivotAngleRads() {
        return inputs.pivotData.position() * ArmConstants.PIVOT_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getExtensionLengthMeters() {
        return inputs.extensionData.position() * ArmConstants.EXTENSION_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getWristAngleRads() {
        return inputs.wristData.position() * ArmConstants.WRIST_P_COEFFICIENT;
    }

    public ArmPosition getArmPosition() {
        return new ArmPosition(getPivotAngleRads(), getExtensionLengthMeters(), getWristAngleRads());
    }

    @AutoLogOutput
    private double getPivotkG() {
        // extension percentage, uses the setpoint to minimize overshoot when both move
        double t = state.position.getExtensionMeters() / ArmState.NET.position.getExtensionMeters();

        return MathUtil.interpolate(ArmConstants.PIVOT_MIN_KG, ArmConstants.PIVOT_MAX_KG, t)
                * Math.cos(state.position.getPivotRads());
    }

    @AutoLogOutput
    private double getExtensionkG() {
        return ArmConstants.EXTENSION_KG * Math.sin(getPivotAngleRads());
    }
}
