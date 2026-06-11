// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final CommandXboxController Xbox = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Articulated Artic = new Articulated();
    public Climb climb = new Climb();
    public Shooter shooter= new Shooter();

    public double intakeVar = 0;
    public double climbMove = 0;

    public double netProtection = 19;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        /* Swerve drive */
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> { return drive
            .withVelocityX(-Xbox.getLeftY() * MaxSpeed * Xbox.getRightTriggerAxis())
            .withVelocityY(-Xbox.getLeftX() * MaxSpeed * Xbox.getRightTriggerAxis())
            .withRotationalRate(-Xbox.getRightX() * MaxSpeed);
        }));

        Xbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.configAngleInit()));
        /* Swerve drive */

        /* Zerar Comandos */

        Trigger voltarIntake = Xbox.b().and(() -> Climb.getPosition() < 10);

        Xbox.b().onTrue(zerarComandos());

        new Trigger(voltarIntake).onTrue(Commands.runOnce(() -> Articulated.setArticulated(0.1, 0, 0.5)));

        /* Zerar comandos */

        /* Intake */

        Xbox.leftBumper().onTrue(Commands.runOnce(() -> intakeVar ++));

        new Trigger(() -> intakeVar == 1).onTrue(intakePreOn());
        new Trigger(() -> intakeVar == 1).onTrue(intakeOn(5000));

        new Trigger(() -> intakeVar >= 2).onTrue(intakeOff());

        /* Intake */

        /* Shooter & ASSISTANCE INDEXER */

        // Gatilho unificado: Right Bumper + Botão X + Shooter Indexando
        Trigger assistIndexerTrigger = Xbox.rightBumper().and(Xbox.x()).and(shooter::getIndexando);

        // 1. Quando a condição iniciar, move a articulação do Intake
        assistIndexerTrigger.onTrue(Commands.runOnce(() -> Articulated.setArticulated(0.015, 0, 0.2)));

        // 2. Se a posição for maior que 19 , liga o rolete do Intake a 2500 RPM
        assistIndexerTrigger.and(() -> Articulated.getArticulatedPosition() > netProtection)
                .onTrue(Commands.runOnce(() -> Artic.setIntakeRPM(2500)));
                
        // 3. Se a posição for menor ou igual ao 19 , desliga o rolete
        assistIndexerTrigger.and(() -> Articulated.getArticulatedPosition() <= netProtection)
                .onTrue(Commands.runOnce(() -> Artic.setIntakeRPM(0)));

        // 4. Ao soltar o botão X, roda o comando de desligar o sistema
        Xbox.x().onFalse(indexerIntakeOff());

        // 5. Nova lógica do Climber baseada na posição da articulação do Intake
        new Trigger(() -> Articulated.getArticulatedPosition() > 2
                        && Articulated.getArticulatedPosition() < 8 
                        && climbMove == 1)
                .onTrue(indexerClimb());

        // Gatilho padrão do disparo do shooter
        Xbox.rightBumper().whileTrue(createShooterCommand());
        Xbox.rightBumper().onFalse(endShooter());

        Trigger dispenser = Xbox.a().and(() -> Articulated.getArticulatedPosition() > netProtection);
        dispenser.onTrue(dispenserOn());
        dispenser.onFalse(dispenserOff());

        /* Shooter */

        /* Climb */

        Xbox.povUp().onTrue(setClimb(98.0));
        Xbox.povDown().onTrue(setClimb(0));
        
        /* Climb */

    }

    /**
     * Move o climb para a posição desejada.
     * @param position Seta a posição desejada do climber.
     * @return Comando que move o climb
     */
    Command setClimb(double position){
        return Commands.runOnce(() -> climb.setPosition(position, 1));
    }

    private Command createShooterCommand() {
    return shooter;
}

    /**
     * Move o climb para a posição de coleta
     */
    Command comandoColeta = Commands.sequence(
        Commands.runOnce(() -> Articulated.setArticulated(0.1, 22.394, 0.5))
        , setClimb(98) 
        ,intakeOn(5000));

    /**
     * Zerar tudo para o standby
     */
    Command zerarComandos(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Shooter.stopShooterSpeed()),
            Commands.runOnce(() -> Shooter.stopIndexSpeed()),
            Commands.runOnce(() -> Shooter.stopBelt()),
            Commands.runOnce(() -> Artic.setIntakeRPM(0)));
    }

    /**
     * Posiciona o climber para baixo.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command indexerClimb(){
        return new SequentialCommandGroup(
            setClimb(0),
            Commands.runOnce(() -> climbMove = 0));
    }

        /**
     * Finaliza os comandos relacionados ao disparo e posiciona o intake para coleta novamente.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */

    Command endShooter(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> shooter.end()),
            Commands.runOnce(() -> Artic.setIntakeRPM(0)),
            Commands.runOnce(() -> Articulated.setArticulated(0.1, 22.394, 0.5))
            );
    }

    Command intakeOn(double RPM){
        return Commands.runOnce(() -> Artic.setIntakeRPM(RPM));
    }

    /**
     * Realiza a coleta de Fuels.
     * @param RPM RPM do intake.
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command intakePreOn(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Articulated.setArticulated(0.1, 22.394, 0.5)),
            setClimb(98));
    }

    Command intakeOff(){
        return new SequentialCommandGroup(
        Commands.runOnce(() -> Artic.setIntakeRPM(0)),
        new InstantCommand(() -> intakeVar = 0)
        );
    }

    /**
     * Realiza sequencia de acionamentos para ejetar o fuel pelo intake.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command dispenserOn(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Artic.setIntakeRPM(-3500)),
            Commands.runOnce(() -> shooter.setBeltRPM(-4000)),
            Commands.runOnce(() -> shooter.setIndexRPM(-3000)),
            Commands.runOnce(() -> shooter.setFeedRPM(-1500)));
    }

    /**
     * Desliga o sistema de ejetar fuel.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command dispenserOff(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Artic.setIntakeRPM(0)),
            Commands.runOnce(() -> shooter.setBeltRPM(0)),
            Commands.runOnce(() -> shooter.setIndexRPM(0)));
    }

    /**
     * Desliga o sistema de auxilio de fuels com intake.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command indexerIntakeOff(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Artic.setIntakeRPM(0)),
            Commands.runOnce(() -> Articulated.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakeVar = 0));
    }

    public Command getAutonomousCommand() {
        throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }
}
