package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
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

    private final indexer index;

    private SmartMotorControllerConfig shooterRConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.4, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withTelemetry("ShooterR_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages( 3 , 4 )))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorControllerConfig shooterLConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.4, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
      .withTelemetry("ShooterL_FlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages( 3, 4)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(80))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController shooterR = new SparkWrapper(mSparkR, DCMotor.getNEO(1), shooterRConfig);

    private SmartMotorController shooterL = new SparkWrapper(mSparkL, DCMotor.getNEO(1), shooterLConfig);

    private final FlyWheelConfig rFlyWheelConfig = new FlyWheelConfig()
    .withDiameter(Inches.of(2))
    .withTelemetry("ShooterR_Flywheel", TelemetryVerbosity.HIGH);

    private final FlyWheelConfig lFlyWheelConfig = new FlyWheelConfig()
    .withDiameter(Inches.of(2))
    .withTelemetry("ShooterL_Flywheel", TelemetryVerbosity.HIGH);

    private FlyWheel rFlyWheel = new FlyWheel(rFlyWheelConfig, shooterR);
    private FlyWheel lFlyWheel = new FlyWheel(lFlyWheelConfig, shooterL);

    /**
     * Construtor nescessario para instanciar o indexer nesse subsistema
     * @param index
     */
    public shooterMotors(indexer index) {
        this.index = index;
    }

    /**
     * Set the AngularVelocity of the both motors in shooter
     * @param upperSpeed
     * @param lowerSpeed
     */
    public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
      return Commands.runOnce(() -> {
          shooterR.setVelocity(upperSpeed);
          shooterL.setVelocity(lowerSpeed);
      }, this);
    }

    /**
     * Set dynamic speeds of motors, is used for recive parameters of InterpolationTreeMap
     * @param rpmSupplier
     */
    public Command setDynamicSpeeds(DoubleSupplier rpmSupplier) {
    return Commands.runOnce(() -> {
        double targetRPM = rpmSupplier.getAsDouble();
        shooterR.setVelocity(RPM.of(targetRPM));
        shooterL.setVelocity(RPM.of(targetRPM));
    }, this);
    }

    /**
     * Stop Motors
     */
    public Command stopMotors(){
      return Commands.runOnce(() -> {
        shooterL.setDutyCycle(0); 
        shooterR.setDutyCycle(0);
      }, this);
    }

    /**
     * Get the velocity of right flywheel for use the rpm in another mechanism
     */
    public double getVelocity(){
      return rFlyWheel.getSpeed().in(RPM);
    }
    
    /**
     * Command for the indexer send the balls for shooter while shootin
     * @values limits of [-1] and [1]
     * @return  velocity of all indexer motors using the shooter based on in fixed values
     */
    public Command indexerWhileShootin() {
            return index.setAllVelocityShootin(
                () -> getVelocity() * 0.5, 
                () -> getVelocity() * 0.7, 
                () -> getVelocity() * 0.5
            );
        }

      @Override
      public void periodic() {
      rFlyWheel.updateTelemetry();
      lFlyWheel.updateTelemetry();
    }

      @Override
      public void simulationPeriodic() {
      lFlyWheel.simIterate();
      rFlyWheel.simIterate();
    }
}