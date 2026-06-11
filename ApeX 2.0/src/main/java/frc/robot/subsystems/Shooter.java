package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut; // <-- NOVO IMPORT
import com.ctre.phoenix6.controls.MotionMagicDutyCycle; // <-- NOVO IMPORT
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Command {
    @SuppressWarnings("unused")
    private static double startTime;
    private static double timerShot;
    private static boolean aligned = false;
    private boolean articulaAux = true;
    private static boolean indexando = false;
    private static boolean alingOk = false;
    private boolean ctrAligned = true;
            
    /* Shooter INIT */
    public static final TalonFX mShooterR = new TalonFX(22);
    public static final TalonFX mShooterL = new TalonFX(21);
    public static final VelocityVoltage shooterControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMShooter = 0.0;
    
    // ====== CORREÇÃO DO OVERRUN ======
    private final DutyCycleOut coastControl = new DutyCycleOut(0); 
    private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(0.2, 0.118, 1.0);
    // =================================
    /* Shooter END */
    
    /* Hood INIT */
    public static TalonFXS mHood = new TalonFXS(23);
    public static final MotionMagicDutyCycle magicCtrHood = new MotionMagicDutyCycle(0); // <-- CORRIGIDO
    /* Hood END */
    
    /* Index INIT */
    public static TalonFX mFeed = new TalonFX(19);
    public static TalonFX mIndex = new TalonFX(20);
    public static final VelocityVoltage indexControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMIndex = 0.0;
    private double targetRPMFeeder = 0.0;
    /* Index END */
    
    /* Belt INIT */
    public static TalonFX mBelt = new TalonFX(18);
    public static final VelocityVoltage beltControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMBelt = 0.0;
    /* Belt END */
    
    public static double OmegaCmd = 0;
    public static double angleTurretSim = 0;
    public static double poseHood = 0, poseHoodSim = 0;
    public double tHigh = 5, tLow = 2, tArticula = 10;
    private boolean RPMShooterOK = false;

    private final InterpolatingTreeMap<Double, ShotParams> shooterLUT = 
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), 
            (start, end, t) -> start.interpolate(end, t)
        );

    PIDController headingPID = new PIDController(0.025, 0.0, 0.001);
    private boolean ctrTimer = true;
    private static Pose2d robotPose = new Pose2d(); // Inicializado para evitar NullPointer


    static {
        configHood(0.5, -0.1, 0.45);
        configIndex(NeutralModeValue.Coast);
        configShooter(0.435, NeutralModeValue.Coast);
        configBelt(0.1, -0.1, 0.1);        
    }

    public Shooter() {
        headingPID.enableContinuousInput(-180, 180);
        headingPID.setTolerance(2);

        shooterLUT.put(1.2, new ShotParams(0.25, 3600)); 
        shooterLUT.put(1.6, new ShotParams(0.40, 3800)); 
        shooterLUT.put(2.0, new ShotParams(0.60, 3900));
        shooterLUT.put(2.4, new ShotParams(0.80, 4100));
        shooterLUT.put(2.8, new ShotParams(1.00, 4150));
        shooterLUT.put(3.2, new ShotParams(1.20, 4200));
        shooterLUT.put(3.6, new ShotParams(1.35, 4250));
        shooterLUT.put(4.0, new ShotParams(1.50, 4300));
        shooterLUT.put(4.4, new ShotParams(1.65, 4375));
        shooterLUT.put(4.8, new ShotParams(1.75, 4500));
        shooterLUT.put(5.2, new ShotParams(1.85, 4700));
        shooterLUT.put(5.6, new ShotParams(1.85, 4950));
    }

    @Override
    public void execute() {
        Translation2d target = determineTargetPosition();
        
        double distanceHood = hoodAling(target.getX(), target.getY());
        
        ShotParams currentParams = calculoShooterParametros(distanceHood);
        
        setHoodPosition(currentParams.hoodPosition);
        setShooterRPM(currentParams.rpm);
        
        RPMShooterOK = checkShooterOk();
        
        controleSequencia(currentParams.rpm);
    }

    /**
     * Verifica qual a equipe e especifica o hub
     * @return Cordenadas do hub
     */
    private Translation2d determineTargetPosition() {
        double blueX = 4.298;   
        double redX = 12.41;    
        double targetX = 0;
        double targetY = 0;

        if ((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)) {
            targetX = isRedAlliance() ? 11.914 : 4.624;
            targetY = 3.915;
        } 
        else {
            targetY = (robotPose.getY() - 4.044) >= 0 ? 6 : 1.829;
            targetX = isRedAlliance() ? 14.32 : 2.331;
        }
        
        return new Translation2d(targetX, targetY);
    }

    private ShotParams calculoShooterParametros(double distanceHood) {
        double blueX = 4.298;   
        double redX = 12.41;

        if ((robotPose.getX() >= blueX - 0.2 && robotPose.getX() <= (blueX - 0.2) + 1.2) || 
            (robotPose.getX() >= (redX + 0.2) - 1.2 && robotPose.getX() <= redX + 0.2)) {
            poseHoodSim = -70;
            return new ShotParams(0, 0); 
        }
        
        // Chute forçado de perto
        if ((isRedAlliance() && robotPose.getX() <= 4.6) || (!isRedAlliance() && robotPose.getX() >= 11.9)) {
            return new ShotParams(1.1, 6000);
        }
        
        return shooterLUT.get(distanceHood);
    }
    /**
     *  Verifica se a margem de erro está dentro de 1000 RPM
     * @return Velocidade do shooter
     */
    private boolean checkShooterOk() {
        double[] velocityShooter = getShooterVelocity();
        return (Math.abs(this.targetRPMShooter - velocityShooter[0]) * 60) < 1000;
    }
    /**
     * Controle para o index, o belt e o shooter funcionarem juntos
     * @param targetRPM
     */
    private void controleSequencia(double targetRPM) {
        double RPMIndex = 0;
        double RPMBelt = 0;

        if (aligned || alingOk) {
            if (ctrAligned) {
                timerShot = Timer.getFPGATimestamp();
                ctrAligned = false;
            }
            
            if (RPMShooterOK && errorTimer(timerShot) > 0.75) {
                if (ctrTimer) {
                    startTime = Timer.getFPGATimestamp();
                    ctrTimer = false;
                }
                RPMIndex = targetRPM * 0.5;
                RPMBelt = 6000;
                indexando = true;
            } else {
                indexando = false;
            }
        } else {
            // Reseta tudo se perder o alinhamento
            indexando = false;
            articulaAux = false;
            startTime = Timer.getFPGATimestamp();
        }

        setIndexer(RPMIndex, RPMBelt);
    }

    public class ShotParams implements Interpolatable<ShotParams> {
        public final double hoodPosition; 
        public final double rpm;          

        public ShotParams(double hoodPosition, double rpm) {
            this.hoodPosition = hoodPosition;
            this.rpm = rpm;
        }

        @Override
        public ShotParams interpolate(ShotParams endValue, double t) {
            return new ShotParams(
                MathUtil.interpolate(this.hoodPosition, endValue.hoodPosition, t),
                MathUtil.interpolate(this.rpm, endValue.rpm, t)
            );
        }
    }

    public void end() {
        setHoodPosition(0);
        
        mShooterL.setControl(coastControl);
        mShooterR.setControl(coastControl);
        
        stopIndexSpeed();
        stopBelt();
    
        aligned=false;
        articulaAux=false;
        indexando = false;
        ctrTimer = true;
        ctrAligned = true;
    }

    public boolean getarticulaAux(){
        return articulaAux && Articulated.getArticulatedPosition() > 9;
    }

    public static void updateRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    private static double errorTimer(double tTarget){
        return Timer.getFPGATimestamp() - tTarget;
    }

    public boolean getIndexando(){
        return indexando;
    }

    private void setIndexer (double RPMIndex, double RPMbelt) {
        setIndexRPM(RPMIndex);
        setFeedRPM(RPMIndex * 0.5);
        setBeltRPM(RPMbelt);
    }

    /**
     * Alinhamento da cremalheira
     * @param Target_X
     * @param Target_Y
     * @return distancia do hood para o hub
     */
    private double hoodAling (double Target_X, double Target_Y){
        Pose2d robot_getValues = robotPose;
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d robot_pose = robot_getValues.getTranslation();

        Translation2d pose_Target = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0.19719, 0);
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));
        double distanceHood = poseHood.getDistance(pose_Target);

        Translation2d angleHoodHub = pose_Target.minus(poseHood);
        Rotation2d targetAngleHood = angleHoodHub.getAngle();

        OmegaCmd = headingPID.calculate(Robot_Yaw.getDegrees(), targetAngleHood.getDegrees());

        OmegaCmd = MathUtil.clamp(OmegaCmd, -0.7, 0.7);
        aligned = Math.abs(headingPID.getError()) < 5;

        SmartDashboard.getNumberArray("ROBOT/OdometryRobot", new double[] {robot_pose.getX(), robot_pose.getY(), Robot_Yaw.getRadians()});

        return distanceHood;
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static void configShooter(double kP, NeutralModeValue kMode) {
        TalonFXConfiguration cfgShooterL = new TalonFXConfiguration();
        TalonFXConfiguration cfgShooterR = new TalonFXConfiguration();

        cfgShooterL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgShooterL.MotorOutput.NeutralMode = kMode;
        cfgShooterL.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterL.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterL.Voltage.PeakForwardVoltage = 12;
        cfgShooterL.Voltage.PeakReverseVoltage = -12;
        cfgShooterL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgShooterL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        cfgShooterL.Slot0.kP = kP;
        cfgShooterL.Slot0.kI = 0.0;
        cfgShooterL.Slot0.kD = 0.0;

        cfgShooterR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgShooterR.MotorOutput.NeutralMode = kMode;
        cfgShooterR.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterR.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterR.Voltage.PeakForwardVoltage = 12;
        cfgShooterR.Voltage.PeakReverseVoltage = -12;
        cfgShooterR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgShooterR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        cfgShooterR.Slot0.kP = kP;
        cfgShooterR.Slot0.kI = 0.0;
        cfgShooterR.Slot0.kD = 0.0;

        mShooterL.getConfigurator().apply(cfgShooterL);
        mShooterR.getConfigurator().apply(cfgShooterR);
    }

    static public double[] getShooterVelocity(){
        return new double[] {
            mShooterL.getVelocity().getValueAsDouble(),
            mShooterR.getVelocity().getValueAsDouble()};
    }

    static public double[] getShooterCurrent(){
        return new double[] {
            mShooterL.getSupplyCurrent().getValueAsDouble(),
            mShooterR.getSupplyCurrent().getValueAsDouble()};
    }
    
    public void setShooterRPM(double setpointRPM){
        if (setpointRPM <= 10) { 
            mShooterL.setControl(coastControl);
            mShooterR.setControl(coastControl);
            this.targetRPMShooter = 0;
            return;
        }

        setpointRPM = MathUtil.clamp(setpointRPM, 0, 6000);
        this.targetRPMShooter = setpointRPM / 60.0;

        double VoltageFeedFoward = shooterFeedforward.calculate(this.targetRPMShooter);

        mShooterL.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
        mShooterR.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
    }

    public static void configHood(double KP, double OutMin, double OutMax) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.85; 
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        cfg.MotionMagic.MotionMagicCruiseVelocity = 8.0; 
        cfg.MotionMagic.MotionMagicAcceleration = 12.0;   
        cfg.MotionMagic.MotionMagicJerk = 40.0;          

        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        cfg.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfg.MotorOutput.PeakReverseDutyCycle = OutMin;

        cfg.Slot0.kP = KP;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        mHood.getConfigurator().apply(cfg);
    }
    
    static public double getHoodPositon() {
        return mHood.getPosition().getValueAsDouble();
    }

    public void setZeroHood(){
            mHood.setPosition(0);
    }

    public void setHoodPosition(double position) {
        position = MathUtil.clamp(position, 0, 1.85);
        mHood.setControl(magicCtrHood.withPosition(position));
    }

    static public double getHoodCurrent(){
        return mHood.getSupplyCurrent().getValueAsDouble();
    }

    public static void configIndex(NeutralModeValue kMode) {
        TalonFXConfiguration cfgFeed = new TalonFXConfiguration();
        TalonFXConfiguration cfgIndex = new TalonFXConfiguration();
        
        cfgFeed.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgFeed.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgFeed.MotorOutput.NeutralMode = kMode;
        cfgFeed.CurrentLimits.SupplyCurrentLimit = 40;
        cfgFeed.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgFeed.Feedback.SensorToMechanismRatio = 1.0;
        cfgFeed.MotorOutput.PeakForwardDutyCycle = 1;
        cfgFeed.MotorOutput.PeakReverseDutyCycle = -1;
        cfgFeed.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgFeed.Voltage.PeakForwardVoltage = 12;
        cfgFeed.Voltage.PeakReverseVoltage = -12;
        cfgFeed.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
        cfgFeed.Slot0.kP = 0.11;
        cfgFeed.Slot0.kI = 0.0;
        cfgFeed.Slot0.kD = 0.0;

        cfgIndex.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgIndex.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgIndex.MotorOutput.NeutralMode = kMode;
        cfgIndex.CurrentLimits.SupplyCurrentLimit = 40;
        cfgIndex.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIndex.Feedback.SensorToMechanismRatio = 1.0;
        cfgIndex.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIndex.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIndex.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIndex.Voltage.PeakForwardVoltage = 12;
        cfgIndex.Voltage.PeakReverseVoltage = -12;
        cfgIndex.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
        cfgIndex.Slot0.kP = 0.11;
        cfgIndex.Slot0.kI = 0.0;
        cfgIndex.Slot0.kD = 0.0;

        mFeed.getConfigurator().apply(cfgFeed);
        mIndex.getConfigurator().apply(cfgIndex);
    }

    static public double[] getIndexVelocity() {
        return new double[] {
            mFeed.getVelocity().getValueAsDouble(),
            mIndex.getVelocity().getValueAsDouble()};
    }

    static public double[] getIndexCurrent() {
        return new double[] {
            mFeed.getSupplyCurrent().getValueAsDouble(),
            mIndex.getSupplyCurrent().getValueAsDouble()};
    }

    public void setIndexRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, -6000, 6000);
        this.targetRPMIndex = setpointRPM / 60.0;
        double kV = 0.15;
        double VoltageFeedFoward = this.targetRPMIndex * kV;
        mIndex.setControl(shooterControl.withVelocity(targetRPMIndex).withFeedForward(VoltageFeedFoward));
    }
    
    public void setFeedRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, 0, 6000);
        this.targetRPMFeeder = setpointRPM / 60.0;
        double kV = 0.15;
        double VoltageFeedFoward = this.targetRPMFeeder * kV;
        mFeed.setControl(shooterControl.withVelocity(targetRPMFeeder).withFeedForward(VoltageFeedFoward));
    }

    static public void stopIndexSpeed() {
        mFeed.stopMotor();
        mIndex.stopMotor();
    }
    
    static void configBelt(double kP, double OutMin, double OutMax){
        TalonFXConfiguration cfgBelt = new TalonFXConfiguration();

        cfgBelt.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgBelt.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgBelt.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgBelt.CurrentLimits.SupplyCurrentLimit = 80;
        cfgBelt.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgBelt.CurrentLimits.SupplyCurrentLimit = 40;
        cfgBelt.Feedback.SensorToMechanismRatio = 1.0;
        cfgBelt.MotorOutput.PeakForwardDutyCycle = 1;
        cfgBelt.MotorOutput.PeakReverseDutyCycle = -1;
        cfgBelt.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgBelt.Voltage.PeakForwardVoltage = 12;
        cfgBelt.Voltage.PeakReverseVoltage = -12;

        mBelt.getConfigurator().apply(cfgBelt);
    }

    static public double getBeltVelocity(){
        return mBelt.getVelocity().getValueAsDouble();
    }

    public void setBeltRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, -6000, 6000);
        this.targetRPMBelt = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPMBelt * kV;
        mBelt.setControl(beltControl.withVelocity(targetRPMBelt).withFeedForward(VoltageFeedFoward));
    }

    /**
    * Para os motores do Shooter e configura o motor em modo coast.
    *
    * @param null
    */
    static public void stopShooterSpeed() {
        mShooterL.stopMotor();
        mShooterR.stopMotor();
    }

    static public void stopBelt(){
        mBelt.stopMotor();
    }

    public static void setAlingAuto(boolean aling){
        alingOk = aling;
    }

    static public double getBeltCurrent(){
        return mBelt.getSupplyCurrent().getValueAsDouble();
    }
}