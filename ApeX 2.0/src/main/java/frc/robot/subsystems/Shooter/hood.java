package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class hood extends SubsystemBase {

    public static final SparkFlex sparkHood = new SparkFlex(7, MotorType.kBrushless);

        private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.8, 0, 0)
            .withTrapezoidalProfile(RPM.of(4000), RotationsPerSecondPerSecond.of(1500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3 , 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(true)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0.1, 0.01, 0.01))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSoftLimits(Degrees.of(-0.5), Degrees.of(55))
            .withStartingPosition(Degrees.of(0));

        private SmartMotorController hoodSm = new SparkWrapper(sparkHood, DCMotor.getNeoVortex(1), hoodMotorConfig);

        private final ArmConfig hoodConfig = new ArmConfig()
                .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
                .withLength(Meters.of(0.1))
                .withHardLimits(Degrees.of(-0.1), Degrees.of(55));

    private final Arm hoodA = new Arm(hoodConfig, hoodSm);

    /**
     * Command that set the angle of hood
     */
    public Command setAngle(Angle angle) {
        return hoodA.setAngle(angle);
    }

    /**
     * Reset the encoder position
     */
    public Command resetAngle() {
    return Commands.runOnce(() -> hoodSm.setEncoderPosition(Degrees.of(0)));
    }

    /**
     * Get the Angle of hood
     */
    public Angle getAngle() {
        return hoodA.getAngle();
    }

    /**
     * Disable hood
     * @return setDutyCycleSetpoint in 0
     */
    public Command hoodOff(){
        return Commands.run(() -> hoodA.setDutyCycleSetpoint(0), this);
    }

    /**
     * Set the hood position using the positionSupplier of InterpolationTreeMap
     * @param positionSupplier
     */
    public Command setHoodPosition(DoubleSupplier positionSupplier) {
        return Commands.run(() -> {
            hoodSm.setPosition(Degrees.of(positionSupplier.getAsDouble()));
        }, this);
    }
    
    @Override
    public void periodic() {
        hoodA.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hoodA.simIterate();
    }
}
