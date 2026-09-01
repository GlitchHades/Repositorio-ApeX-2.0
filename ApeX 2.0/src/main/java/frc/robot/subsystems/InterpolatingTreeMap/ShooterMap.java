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

        lut.put(1.2, new ShotParams(10.00, 3600)); 
        lut.put(1.6, new ShotParams(14.75, 3800)); 
        lut.put(2.0, new ShotParams(18.81, 3900));
        lut.put(2.4, new ShotParams(19.49, 4100));
        lut.put(2.8, new ShotParams(20.17, 4150));
        lut.put(3.2, new ShotParams(20.85, 4200));
        lut.put(3.6, new ShotParams(21.36, 4250));
        lut.put(4.0, new ShotParams(25.25, 4300));
        lut.put(4.4, new ShotParams(29.15, 4375));
        lut.put(4.8, new ShotParams(32.88, 4500));
        lut.put(5.2, new ShotParams(36.61, 4700));
        lut.put(5.6, new ShotParams(40.00, 4950));
    }

    /**
     * Retorna os parâmetros de tiro ideais para uma dada distância.
     */
    public static ShotParams get(double distanceInMeters) {
        return lut.get(distanceInMeters);
    }
}