// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 


import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Articulated Artic = new Articulated();
    public Climb climb = new Climb();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> { return drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed * joystick.getRightTriggerAxis())
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * joystick.getRightTriggerAxis())
            .withRotationalRate(-joystick.getRightX() * MaxSpeed);
        }));

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.configAngleInit()));
        joystick.leftBumper().onTrue(comandoColeta);
        joystick.b().onTrue(zerarComandos);
    }

    /**
     * Move o climb para a posição desejada.
     * @param position Seta a posição desejada do climber.
     * @return Comando que move o climb
     */
    Command setClimb(double position){
        return Commands.runOnce(() -> climb.setPosition(position, 1));
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
    Command zerarComandos = Commands.sequence(Commands.runOnce(() -> Articulated.setArticulated(0.1, 0, 0.5))
    ,setClimb(0)
    ,intakeOn(0));

    Command intakeOn(double RPM){
        return Commands.runOnce(() -> Artic.setIntakeRPM(RPM));
    }

    public Command getAutonomousCommand() {
        throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }
}
