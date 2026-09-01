package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class intake extends SubsystemBase {

        public static final SparkMax sparkIntake = new SparkMax(44, MotorType.kBrushless);

        private final SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1, 0, 0)
            .withTrapezoidalProfile(RPM.of(4000), RotationsPerSecondPerSecond.of(400))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 1)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(true)
            .withClosedLoopRampRate(Seconds.of(0.5))
            .withOpenLoopRampRate(Seconds.of(0.5))
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.1, 0.1))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSoftLimits(Degrees.of(0), Degrees.of(180))
            .withStartingPosition(Degrees.of(1));

        private SmartMotorController intakeArticulatedArm = new SparkWrapper(sparkIntake, DCMotor.getNEO(1), intakeMotorConfig);

        private final ArmConfig intakeConfig = new ArmConfig()
                .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
                .withLength(Meters.of(0.32))
                .withHardLimits(Degrees.of(0), Degrees.of(180));

    private final Arm intakeArm = new Arm(intakeConfig, intakeArticulatedArm);

    /**
     * Defines the setpoint of the arm angle
     * @param angle
     * @return Command that sets the arm to the desired angle.
     */
    public Command setAngle(Angle angle) {
        return intakeArm.setAngle(angle);
    }

    /**
     * @return Angle of arm in degrees
     */
    public double getAngle() {
        return intakeArm.getAngle().in(Degrees);
    }
    
    /**
     * Command that set where the intake arm will go
     * @param degrees
     */
    public Command deployIntake(double degrees) {
        return setAngle(Degrees.of(degrees)); 
    }

    /**
     * Command that retract the intake to zero
     * @see only use that command if you will set the dutyCycle in zero after he
     */
    public Command retract() {
        return setAngle(Degrees.of(0)); 
    }

    /**
     * @return setDutyCycle in zero
     */
    public void zeroDutyCycle(){
        intakeArm.setDutyCycleSetpoint(0);
    }

    @Override
    public void periodic() {
        intakeArm.updateTelemetry();
    }
    
    @Override
    public void simulationPeriodic() {
        intakeArm.simIterate();
    }
}
