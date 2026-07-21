package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class intake extends SubsystemBase {

        public static final SparkMax sparkIntake = new SparkMax(44, MotorType.kBrushless);
        public static final SparkMax sparkIntakeL = new SparkMax(45, MotorType.kBrushless);
        public static final SparkMax sparkIntakeR = new SparkMax(46, MotorType.kBrushless);

        private final SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1.5, 0, 0)
            .withTrapezoidalProfile(RPM.of(4000), RotationsPerSecondPerSecond.of(1500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1))
            .withFeedforward(new SimpleMotorFeedforward(0.2, 0.1, 0.1))
            .withSimFeedforward(new SimpleMotorFeedforward(0.2, 0.1, 0.1))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSoftLimits(Degrees.of(0), Degrees.of(180))
            .withStartingPosition(Degrees.of(0));

        private SmartMotorControllerConfig intakeRConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // PID Constants for velocity control
            .withClosedLoopController(0.4, 0, 0)
            .withSimClosedLoopController(0.1, 0, 0)
            // Feedforward Constants - helps track changing RPM goals
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("intakeR_FlyWheel", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft (3:1 reduction)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            // Motor properties
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80))
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1));

        private SmartMotorControllerConfig intakeLConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // PID Constants for velocity control
            .withClosedLoopController(0.4, 0, 0)
            .withSimClosedLoopController(0.1, 0, 0)
            // Feedforward Constants - helps track changing RPM goals
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("intakeL_FlyWheel", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft (3:1 reduction)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            // Motor properties
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80))
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1));

        private SmartMotorController intakeR = new SparkWrapper(sparkIntakeR, DCMotor.getNEO(1), intakeRConfig);

        private SmartMotorController intakeL = new SparkWrapper(sparkIntakeL, DCMotor.getNEO(1), intakeLConfig);

        private SmartMotorController intakeArticulatedArm = new SparkWrapper(sparkIntake, DCMotor.getNEO(1), intakeMotorConfig);

        private final ArmConfig intakeConfig = new ArmConfig()
                .withSmartMotorController(intakeArticulatedArm)
                .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
                .withLength(Meters.of(0.25))
                .withHardLimits(Degrees.of(0), Degrees.of(180));

        private final FlyWheelConfig rFlyWheelConfig = new FlyWheelConfig()
            .withSmartMotorController(intakeR)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.5))
            .withTelemetry("intakeR_Flywheel", TelemetryVerbosity.HIGH);

        private final FlyWheelConfig lFlyWheelConfig = new FlyWheelConfig()
            .withSmartMotorController(intakeL)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.5))
            .withTelemetry("intakeL_Flywheel", TelemetryVerbosity.HIGH);

    private final Arm intakeArm = new Arm(intakeConfig);
    private FlyWheel rIntakeFlyWheel = new FlyWheel(rFlyWheelConfig);
    private FlyWheel lIntakeFlyWheel = new FlyWheel(lFlyWheelConfig);

    public Command setAngle(Angle angle) {
        return intakeArm.setAngle(angle);
    }

    public double getAngle() {
        return intakeArm.getAngle().in(Degrees);
    }
    
    public Command setIntakePosition(double positionSupplier) {
        return Commands.run(() -> {
            intakeArticulatedArm.setPosition(Degrees.of(positionSupplier));
        }, this);
    }
    
    public Command deployIntake(double positionRotations, double rpmL, double rpmR) {
        return Commands.run(() -> {
            intakeArticulatedArm.setPosition(Degrees.of(positionRotations));
            intakeL.setVelocity(RPM.of(rpmL));
            intakeR.setVelocity(RPM.of(rpmR));
        }, this);
    }

    ///////////////////////// MOTORES DE ROTAÇÃO /////////////////////////


    public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
      return Commands.run(() -> {
          intakeR.setVelocity(upperSpeed);
          intakeL.setVelocity(lowerSpeed);
      }, this);
    }

    public Command setDynamicIntakeSpeeds(DoubleSupplier rpmSupplier) {
    return Commands.run(() -> {
        double targetRPM = rpmSupplier.getAsDouble();
        intakeL.setVelocity(RPM.of(targetRPM));
        intakeR.setVelocity(RPM.of(targetRPM));
    }, this);
    }
       
    public Command setRPMintake(double intakeR, double intakeL) {
      return setFlywheelSpeeds(RPM.of(intakeR), RPM.of(intakeL));
    }

    public Command stopMotors(){
      return setFlywheelSpeeds(RPM.of(0), RPM.of(0));
    }

    @Override
    public void periodic() {
        intakeArm.updateTelemetry();
        rIntakeFlyWheel.updateTelemetry();
        lIntakeFlyWheel.updateTelemetry();
    }
    
    @Override
    public void simulationPeriodic() {
        intakeArm.simIterate();
        rIntakeFlyWheel.simIterate();
        lIntakeFlyWheel.simIterate();
    }
}
