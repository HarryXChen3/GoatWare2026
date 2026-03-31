package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class CRT {
//    private static long[] egcd(final long a, final long b) {
//        long ma = a;
//        long mb = b;
//
//        long x = 0;
//        long y = 1;
//        long lastX = 1, lastY = 0;
//
//        int i = 0;
//        while (mb != 0) {
//            if (i > 100) {
//                throw new RuntimeException("EGCD exceeded iteration limit");
//            }
//
//            final long f = ma / mb;
//            final long remainder = ma % mb;
//            ma = mb;
//            mb = remainder;
//
//            final long tx = x;
//            x = lastX - (f * x);
//            lastX = tx;
//
//            final long ty = y;
//            y = lastY - (f * y);
//            lastY = ty;
//
//            i++;
//        }
//
//        return new long[] {ma, lastX, lastY};
//    }
//
//    private static long mod(final long x, final long m) {
//        final long r = x % m;
//        return r < 0 ? r + m : r;
//    }
//
//    private static long crt(final long a, final long m, final long b, final long n) {
//        final long ma = mod(a, m);
//        long mb = mod(b, n);
//
//        long[] vals = egcd(m, n);
//        long g = vals[0];
//        long x = vals[1];
//
//        long diff = mod(mb - ma, g);
//        if (diff != 0) {
//            mb -= diff;
//        }
//
//        long lcm = (m / g) * n;
//
//        long modN = n / g;
//        long k = (mb - ma) / g;
//        k = mod(k, modN);
//
//        return mod(ma + m * mod(k * x, modN), lcm);
//    }
//
//    private static double mod1(double x) {
//        double r = x % 1.0;
//        return r < 0 ? r + 1.0 : r;
//    }
//
//    public static Rotation2d findAbsolutePosition(
//            final int outputGearTeeth,
//            final double absolutePosition0,
//            final int gearTeeth0,
//            final double absolutePosition1,
//            final int gearTeeth1
//    ) {
//        final double pos0 = mod1(absolutePosition0);
//        final double pos1 = mod1(absolutePosition1);
//
//        final long a = mod(Math.round(pos0 * gearTeeth0), gearTeeth0);
//        final long b = mod(Math.round(pos1 * gearTeeth1), gearTeeth1);
//
//        return Rotation2d.fromRotations((double) crt(a, gearTeeth0, b, gearTeeth1) / outputGearTeeth);
//    }

    public static long[] bezout(final long a, final long b) {
        long old_r = a, r = b;
        long old_s = 1, s = 0;
        long old_t = 0, t = 1;

        while (r != 0) {
            long quotient = old_r / r;

            long temp_r = r;
            r = old_r - quotient * r;
            old_r = temp_r;

            long temp_s = s;
            s = old_s - quotient * s;
            old_s = temp_s;

            long temp_t = t;
            t = old_t - quotient * t;
            old_t = temp_t;
        }

        return new long[]{old_s, old_t, old_r};
    }

    public static double crt(final double rawA, final int m, final double rawB, final int n) {
        final long[] bez = bezout(m, n);
        final long u = bez[0];
        final long v = bez[1];
        final long gcd = bez[2];

        double a = rawA * m;
        double b = rawB * n;

        final double offset = MathUtil.inputModulus(a - b, -0.5, 0.5);
        a -= 0.5 * offset;
        b += 0.5 * offset;

        final double q = ((double)m * n) / gcd;
        final double x = ((a * v * n) + (b * u * m)) / gcd;

        return x % q;
    }

    public static Rotation2d findAbsolutePosition(
            final int outputGearTeeth,
            final double absolutePosition0,
            final int gearTeeth0,
            final double absolutePosition1,
            final int gearTeeth1
    ) {
        return Rotation2d.fromRotations(
                crt(absolutePosition0, gearTeeth0, absolutePosition1, gearTeeth1) / outputGearTeeth
        );
    }
}
