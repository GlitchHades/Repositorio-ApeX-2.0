package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

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

public class intakeFlyheel extends SubsystemBase{

        public static final SparkMax sparkIntakeL = new SparkMax(45, MotorType.kBrushless);
        public static final SparkMax sparkIntakeR = new SparkMax(46, MotorType.kBrushless);

        private SmartMotorControllerConfig intakeRConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
            .withTelemetry("intakeR_FlyWheel", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80))
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1));

        private SmartMotorControllerConfig intakeLConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
            .withTelemetry("intakeL_FlyWheel", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1));

        private SmartMotorController intakeR = new SparkWrapper(sparkIntakeR, DCMotor.getNEO(1), intakeRConfig);

        private SmartMotorController intakeL = new SparkWrapper(sparkIntakeL, DCMotor.getNEO(1), intakeLConfig);

        private final FlyWheelConfig rFlyWheelConfig = new FlyWheelConfig()
            .withDiameter(Inches.of(2))
            .withTelemetry("intakeR_Flywheel", TelemetryVerbosity.HIGH);

        private final FlyWheelConfig lFlyWheelConfig = new FlyWheelConfig()
            .withDiameter(Inches.of(2))
            .withTelemetry("intakeL_Flywheel", TelemetryVerbosity.HIGH);

    private FlyWheel rIntakeFlyWheel = new FlyWheel(rFlyWheelConfig, intakeR);
    private FlyWheel lIntakeFlyWheel = new FlyWheel(lFlyWheelConfig, intakeL);

    /**
     * Set the AngularVelocity of the both motors in intakeFlyheel
     * @param upperSpeed
     * @param lowerSpeed
     */
    public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
      return Commands.runOnce(() -> {
          intakeR.setVelocity(upperSpeed);
          intakeL.setVelocity(lowerSpeed);
      });
    }

    /**
     * @param intakeR
     * @param intakeL
     * @return Set RPM of intake
     */
    public Command setRPMintake(double intakeR, double intakeL) {
      return setFlywheelSpeeds(RPM.of(intakeR), RPM.of(intakeL));
    }

    /**
     * @return setDutyCycleSetpoint in 0
     */
    public Command stopMotors(){
      return Commands.runOnce(() -> {
        intakeL.setDutyCycle(0);
        intakeR.setDutyCycle(0);
        }, this);
    }

    @Override
    public void periodic() {
        rIntakeFlyWheel.updateTelemetry();
        lIntakeFlyWheel.updateTelemetry();
    }
    
    @Override
    public void simulationPeriodic() {
        rIntakeFlyWheel.simIterate();
        lIntakeFlyWheel.simIterate();
    }
}
