package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class indexer extends SubsystemBase {

    public static final SparkMax sparkIndex = new SparkMax(42, MotorType.kBrushless);
    public static final SparkMax sparkFeeder = new SparkMax(41, MotorType.kBrushless);
    public static final SparkMax sparkBelt = new SparkMax(40, MotorType.kBrushless);

    private SmartMotorControllerConfig indexControllerConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(1, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0., 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
      .withTelemetry("Indexer_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorControllerConfig feederControllerConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.2, 0, 0)
      .withSimClosedLoopController(0.2, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0., 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("feeder_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorControllerConfig beltControllerConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.2, 0, 0)
      .withSimClosedLoopController(0.2, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0., 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
      .withTelemetry("belt_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController indexer = new SparkWrapper(sparkIndex, DCMotor.getNEO(1), indexControllerConfig);
    private SmartMotorController belt = new SparkWrapper(sparkBelt, DCMotor.getNEO(1), beltControllerConfig);
    private SmartMotorController feeder = new SparkWrapper(sparkFeeder, DCMotor.getNEO(1), feederControllerConfig);

    private final FlyWheelConfig indexFlyWheelConfig = new FlyWheelConfig() 
    .withSmartMotorController(indexer) 
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withTelemetry("Indexer_Flywheel", TelemetryVerbosity.HIGH);

        
    private final FlyWheelConfig feederFlyWheelConfig = new FlyWheelConfig() 
    .withSmartMotorController(feeder) 
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withTelemetry("feeder_Flywheel", TelemetryVerbosity.HIGH);

    private final FlyWheelConfig beltFlyWheelConfig = new FlyWheelConfig() 
    .withSmartMotorController(belt) 
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withTelemetry("belt_Flywheel", TelemetryVerbosity.HIGH);
    
    private FlyWheel indexFlyWheel = new FlyWheel(indexFlyWheelConfig);
    private FlyWheel feederFlyWheel = new FlyWheel(feederFlyWheelConfig);
    private FlyWheel beltFlyWheel = new FlyWheel(beltFlyWheelConfig);

    public Command setIndexDutyCycle(double DutyCycle) {
        return Commands.run (() -> indexFlyWheel.setDutyCycleSetpoint(DutyCycle), this);
    }

    public Command setFlywheelSpeeds(AngularVelocity fVelocity, AngularVelocity bVelocity, AngularVelocity iVelocity) {
        return Commands.run(() -> {
            indexer.setVelocity(iVelocity);
            feeder.setVelocity(fVelocity);
            belt.setVelocity(bVelocity);
        }, this);
    }

    public Command setVelocity(double rpmDesejado){
        return indexFlyWheel.run(RPM.of(rpmDesejado));
    }

    public Command setBothVelocity(double feeder,double belt,double indexer){
        return setFlywheelSpeeds(RPM.of(-feeder), RPM.of(belt), RPM.of(indexer));
    }

    public Command setBothVelocityShootin(DoubleSupplier feederRate, DoubleSupplier beltRate, DoubleSupplier indexerRate) {
        return Commands.run(() -> {
            feeder.setVelocity(RPM.of(-feederRate.getAsDouble()));
            belt.setVelocity(RPM.of(beltRate.getAsDouble()));
            
            indexer.setVelocity(RPM.of(indexerRate.getAsDouble()));
        }, this);
    }

    public Command stopMotors(){
        return setBothVelocity(0, 0, 0);
    }

    @Override
    public void periodic() {
    indexFlyWheel.updateTelemetry();
    feederFlyWheel.updateTelemetry();
    beltFlyWheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
    indexFlyWheel.simIterate();
    feederFlyWheel.simIterate();
    beltFlyWheel.simIterate();
    }
}
