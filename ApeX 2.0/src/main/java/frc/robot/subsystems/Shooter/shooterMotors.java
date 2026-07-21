package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import yams.motorcontrollers.SmartMotorController;
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
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class shooterMotors extends SubsystemBase {

    public static final SparkMax mSparkR = new SparkMax(22, MotorType.kBrushless);
    public static final SparkMax mSparkL = new SparkMax(21, MotorType.kBrushless);

    // Recebido por injeção de dependência: usa a MESMA instância que o resto
    // do robô usa, ao invés de criar um FlyWheel/SmartMotorController paralelo
    // controlando o mesmo hardware físico (sparkIndex/sparkFeeder/sparkBelt são static).
    private final indexer index;

    private SmartMotorControllerConfig shooterRConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.4, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withTelemetry("ShooterR_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorControllerConfig shooterLConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.4, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withTelemetry("ShooterL_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController shooterR = new SparkWrapper(mSparkR, DCMotor.getNEO(1), shooterRConfig);

    private SmartMotorController shooterL = new SparkWrapper(mSparkL, DCMotor.getNEO(1), shooterLConfig);

    private final FlyWheelConfig rFlyWheelConfig = new FlyWheelConfig()
    .withSmartMotorController(shooterR)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withTelemetry("ShooterR_Flywheel", TelemetryVerbosity.HIGH);

    private final FlyWheelConfig lFlyWheelConfig = new FlyWheelConfig()
    .withSmartMotorController(shooterL)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(0.5))
    .withTelemetry("ShooterL_Flywheel", TelemetryVerbosity.HIGH);

    private FlyWheel rFlyWheel = new FlyWheel(rFlyWheelConfig);
    private FlyWheel lFlyWheel = new FlyWheel(lFlyWheelConfig);

    public shooterMotors(indexer index) {
        this.index = index;
    }

    public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
      return Commands.run(() -> {
          shooterR.setVelocity(upperSpeed);
          shooterL.setVelocity(lowerSpeed);
      }, this);
    }

    public double getVelocity(){
      return rFlyWheel.getSpeed().in(RPM);
    }
    
    public Command setDynamicSpeeds(DoubleSupplier rpmSupplier) {
    return Commands.run(() -> {
        double targetRPM = rpmSupplier.getAsDouble();
        shooterR.setVelocity(RPM.of(targetRPM));
        shooterL.setVelocity(RPM.of(targetRPM));
    }, this);
    }

    public Command setShooterRPM(double rightRPM, double leftRPM) {
      return setFlywheelSpeeds(RPM.of(rightRPM), RPM.of(leftRPM));
    }

    public Command setDutyCycle(double DutyCycle){
      return Commands.run (() -> rFlyWheel.setDutyCycleSetpoint(DutyCycle), this);
    }

    public Command stopMotors(){
      return setFlywheelSpeeds(RPM.of(0), RPM.of(0));
    }

    public Command indexerWhileShootin() {
            return index.setBothVelocityShootin(
                () -> getVelocity() * 0.5, 
                () -> getVelocity() * 0.7, 
                () -> getVelocity() * 0.5
            );
        }

      @Override
      public void periodic() {
      // Update telemetry for both flywheels
      rFlyWheel.updateTelemetry();
      lFlyWheel.updateTelemetry();
    }

      @Override
      public void simulationPeriodic() {
      lFlyWheel.simIterate();
      rFlyWheel.simIterate();
    }
}