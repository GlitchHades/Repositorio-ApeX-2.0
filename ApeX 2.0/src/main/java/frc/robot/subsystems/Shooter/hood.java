package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    public static final SparkMax sparkHood = new SparkMax(43, MotorType.kBrushless);

        private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.1, 0, 0)
            .withTrapezoidalProfile(RPM.of(4000), RotationsPerSecondPerSecond.of(1500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1))
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.01))
            .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.01))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSoftLimits(Degrees.of(0), Degrees.of(2))
            .withStartingPosition(Degrees.of(0));

        private SmartMotorController hoodSm = new SparkWrapper(sparkHood, DCMotor.getNEO(1), hoodMotorConfig);

        private final ArmConfig hoodConfig = new ArmConfig()
                .withSmartMotorController(hoodSm)
                .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
                .withLength(Meters.of(0.1))
                .withHardLimits(Degrees.of(0), Degrees.of(2));

    private final Arm hood = new Arm(hoodConfig);

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command setHoodPosition(DoubleSupplier positionSupplier) {
        return Commands.run(() -> {
            hoodSm.setPosition(Degrees.of(positionSupplier.getAsDouble()));
        }, this);
    }
    
    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
