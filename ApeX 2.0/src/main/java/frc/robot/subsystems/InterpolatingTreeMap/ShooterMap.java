package frc.robot.subsystems.InterpolatingTreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterMap {

    public static class ShotParams implements Interpolatable<ShotParams> {
        public final double hoodPosition; 
        public final double rpm;          

        public ShotParams(double hoodPosition, double rpm) {
            this.hoodPosition = hoodPosition;
            this.rpm = rpm;
        }

        @Override
        public ShotParams interpolate(ShotParams endValue, double t) {
            double interpolatedHood = this.hoodPosition + (endValue.hoodPosition - this.hoodPosition) * t;
            double interpolatedRPM = this.rpm + (endValue.rpm - this.rpm) * t;
            
            return new ShotParams(interpolatedHood, interpolatedRPM);
        }
    }

    private static final InterpolatingTreeMap<Double, ShotParams> lut = 
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), 
            ShotParams::interpolate
        );

    // Bloco estático para preencher a tabela quando o código iniciar
    static {
        /*lut.put(1.2, new ShotParams(0.25, 3600)); 
        lut.put(1.6, new ShotParams(0.40, 3800)); 
        lut.put(2.0, new ShotParams(0.60, 3900));
        lut.put(2.4, new ShotParams(0.80, 4100));
        lut.put(2.8, new ShotParams(1.00, 4150));
        lut.put(3.2, new ShotParams(1.20, 4200));
        lut.put(3.6, new ShotParams(1.35, 4250));
        lut.put(4.0, new ShotParams(1.50, 4300));
        lut.put(4.4, new ShotParams(1.65, 4375));
        lut.put(4.8, new ShotParams(1.75, 4500));
        lut.put(5.2, new ShotParams(1.85, 4700));
        lut.put(5.6, new ShotParams(1.85, 4950));*/

        lut.put(1.2, new ShotParams(1, 3600)); 
        lut.put(1.6, new ShotParams(1.40, 3800)); 
        lut.put(2.0, new ShotParams(1.60, 3900));
        lut.put(2.4, new ShotParams(1.80, 4100));
        lut.put(2.8, new ShotParams(2.00, 4150));
        lut.put(3.2, new ShotParams(2.20, 4200));
        lut.put(3.6, new ShotParams(2.35, 4250));
        lut.put(4.0, new ShotParams(2.50, 4300));
        lut.put(4.4, new ShotParams(2.65, 4375));
        lut.put(4.8, new ShotParams(2.75, 4500));
        lut.put(5.2, new ShotParams(2.85, 4700));
        lut.put(5.6, new ShotParams(2.85, 4950));   
    }

    /**
     * Retorna os parâmetros de tiro ideais para uma dada distância.
     */
    public static ShotParams get(double distanceInMeters) {
        return lut.get(distanceInMeters);
    }
}