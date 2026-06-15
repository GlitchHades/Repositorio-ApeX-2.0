package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb  extends Command {

    public static TalonFXS mclimber = new TalonFXS(17);
    public static PositionDutyCycle PID = new PositionDutyCycle(0);


    public Climb(){
        configClimber(0.1, -1, 1, NeutralModeValue.Brake);
    }

    /**
    * Configura o climb.
    * @motor @param Type 1 x Minion
    *
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Velocidade maximo e minima do sistema [LIMITE = -1].
    * @param OutMax Velocidade maximo e minima do sistema [LIMITE = 1].
    * @param kMode Define o freio do motor, brake ou coast.
    */
    static void configClimber(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = kMode;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.MotorOutput.PeakForwardDutyCycle = OutMax;
        config.MotorOutput.PeakReverseDutyCycle = OutMin;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = KP;
        slot0.kI = 0.0;
        slot0.kD = 0;

        mclimber.getConfigurator().apply(config);
    }

    /**
    * Retorna a posição do climber.
    *
    */
    public static double getPosition() {
        return mclimber.getPosition().getValueAsDouble();
    }

    public void setZeroClimber(){
        mclimber.setPosition(0);
    }

    /**
    * Define a posição do Climber.
    *
    * @param position Posição desejada do Climber.
    * @param speed Velocidade maximo e minima do sistema [LIMITES 1 a -1].
    */
    public void setPosition(double position, double speed) {

        position = MathUtil.clamp(position, 0, 110);

        mclimber.setControl(PID.withPosition(position));
        
    }

    /**
    * Para o motor do Climber.
    */
    static public void stop(){
        configClimber(0, 0, 0, NeutralModeValue.Coast);
        mclimber.set(0);
    }
}
