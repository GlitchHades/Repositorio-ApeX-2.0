// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.intake;
import frc.robot.subsystems.Intake.intakeFlyheel;
import frc.robot.subsystems.InterpolatingTreeMap.ShooterMap;
import frc.robot.subsystems.Shooter.hood;
import frc.robot.subsystems.Shooter.indexer;
import frc.robot.subsystems.Shooter.shooterMotors;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
    private final CommandXboxController Xbox = new CommandXboxController(0);

    private final PIDController headingPid = new PIDController(8.0, 0.0, 0.0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public hood Hood = new hood();
    public indexer index = new indexer();
    public shooterMotors shooter = new shooterMotors(index);    
    public intake intake = new intake();
    public intakeFlyheel intakeFly = new intakeFlyheel();

    public double intakeState = 0;
    public double climbMove = 0;

    private Rotation2d lockedHeading = new Rotation2d(); 
    private boolean isHeadingLocked = false;

    @SuppressWarnings("unused")
    private boolean isAimLocked = false;
    public double intakeCheckpoint = 160;

    private ShooterMap.ShotParams calculoShooterParametros(Pose2d robotPose, double distanceHood) {
        double blueX = 4.298;   
        double redX = 12.41;

        if ((robotPose.getX() >= blueX - 0.2 && robotPose.getX() <= (blueX - 0.2) + 1.2) || 
            (robotPose.getX() >= (redX + 0.2) - 1.2 && robotPose.getX() <= redX + 0.2)) {
            return new ShooterMap.ShotParams(0, 0);
        }
        
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if ((isRed && robotPose.getX() <= 4.6) || (!isRed && robotPose.getX() >= 11.9)) {
            return new ShooterMap.ShotParams(1, 6000);
        }
        
        return ShooterMap.get(distanceHood);
    }

    private ShooterMap.ShotParams getParametrosDeTiroAtuais() { 

        Pose2d robotPose = drivetrain.getState().Pose; 

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub = isRed ? new Translation2d(12.05, 4.03) : new Translation2d(4.6, 4.03);

        double distanciaAteOAlvo = robotPose.getTranslation().getDistance(hub);

        ShooterMap.ShotParams params = calculoShooterParametros(robotPose, distanciaAteOAlvo);

        System.out.println("RobotX=" + robotPose.getX() + " dist=" + distanciaAteOAlvo + " hoodPos=" + params.hoodPosition);

        return params;
    }   

    public RobotContainer() {
        headingPid.enableContinuousInput(-Math.PI, Math.PI);
        headingPid.setTolerance(Math.toRadians(0.5));
        configureBindings();
    }

    private void configureBindings() {

        /* Swerve drive */
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {

        double R2 = Xbox.getRightTriggerAxis();
        double leftY = -Xbox.getLeftY();
        double leftX = -Xbox.getLeftX();
        double rightX = -Xbox.getRightX();
        double rotOutput = 0.0;

            Pose2d poseAtual = drivetrain.getState().Pose;

            double vxField = leftY * MaxSpeed * R2;
            double vyField = leftX * MaxSpeed * R2;

            rotOutput = rightX;

            if (Xbox.rightBumper().getAsBoolean()) {
                isAimLocked = false;
                isHeadingLocked = false; 

                boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
                double alvoX = isRed ? 12.05 : 4.6;
                double alvoY = isRed ? 4.03 : 4.03;

                double diferencaX = alvoX - poseAtual.getX();
                double diferencaY = alvoY - poseAtual.getY();

                Rotation2d anguloProHubContinuo = new Rotation2d(diferencaX, diferencaY);

                double calcVisao = headingPid.calculate(poseAtual.getRotation().getRadians(), anguloProHubContinuo.getRadians());

                rotOutput = headingPid.atSetpoint() ? 0.0 : calcVisao;
                rotOutput = MathUtil.clamp(rotOutput, -0.7, 0.7);
            }

            else if (headingPid.atSetpoint() == true) {
                isAimLocked = true;
            }

            else if (Math.abs(rightX) > 0) {
                isAimLocked = false; 
                    
                if (isHeadingLocked) {
                    isHeadingLocked = false;
                    headingPid.reset(); 
                }

                rotOutput = rightX * MaxAngularRate;
                }
            else {
                isAimLocked = false; 
                    
                if (!isHeadingLocked) {
                    lockedHeading = poseAtual.getRotation();
                    headingPid.reset(); 
                    isHeadingLocked = true;
                }
                    
                double calcTrava = headingPid.calculate(
                    poseAtual.getRotation().getRadians(),
                    lockedHeading.getRadians()
                );

                rotOutput = headingPid.atSetpoint() ? 0.0 : calcTrava;
                rotOutput = MathUtil.clamp(rotOutput, -0.7, 0.7);
                }

            return drive
            .withVelocityX(vxField)
            .withVelocityY(vyField)
            .withRotationalRate(rotOutput);
        }));

        Xbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.configurarAnguloInicial()));
        /* Swerve drive */

        Xbox.b().onTrue(Hood.setAngle(Degrees.of(15)));
        
        Xbox.y().onTrue(Hood.hoodOff());

    }

    Command autoMiraEPrepara = Commands.parallel(
        shooter.setDynamicSpeeds(() -> getParametrosDeTiroAtuais().rpm),
        Hood.setHoodPosition(() -> getParametrosDeTiroAtuais().hoodPosition)
    );

    Command BasePosition(){
        return Commands.parallel(
            goBack(),
            Hood.hoodOff(),
            shooter.stopMotors(),
            Commands.runOnce(() -> intakeState = 0),
            index.stopMotors());
    }

    Command retract(){
        return Commands.parallel(
            goBack(),
            intakeFly.setFlywheelSpeeds(RPM.of(0), RPM.of(0))
            );
    }

    Command goBack(){
        return Commands.parallel(
            intake.retract()
            .until(() -> intakeCheckpoint < 30)
            .finallyDo(() -> intake.zeroDutyCycle()));
    }
    
    @SuppressWarnings("unused")
    private void triggerCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);}

    Command getAutonomousCommand() {
        return Commands.none();
    }
}
