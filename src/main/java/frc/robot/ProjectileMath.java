/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;
import static java.lang.Math.*;

//https://www.researchgate.net/publication/215869855_Approximate_analytical_investigation_of_projectile_motion_in_a_medium_with_quadratic_drag_force
public class ProjectileMath {
    private final static int kNMinLoops = 5;

    private static double _cos2(double th) {
        double x = cos(th);
        return x * x;
    }

    private static double _sin2(double th) {
        double x = sin(th);
        return x * x;
    }

    private static double _sqr(double x) {
        return x * x;
    }

    private static double _f(double th) {
        return sin(th) / _cos2(th) + log(tan(th / 2 + PI / 4));
    }

    private static double _v(double th, double th_0, double v_0, double k) {
        double v_0_2 = v_0 * v_0;
        double sqrtP = 1 + k * v_0_2 * _cos2(th_0) * (_f(th_0) - _f(th));
        double num = v_0 * cos(th_0);
        double denom = cos(th) * sqrt(sqrtP);
        return num / denom;
    }

    private static double _v2(double th, double th_0, double v_0, double k) {
        double x = _v(th, th_0, v_0, k);
        return x * x;
    }

    private static double _eps(double th, double th_0, double v_0, double k) {
        double v_0_2 = v_0 * v_0;
        return k * (v_0_2 * sin(th_0) + _v2(th, th_0, v_0, k) * sin(th));
    }

    public static double _y(double th, double th_0, double v_0, double k, double g) {
        double v_0_2 = v_0 * v_0;
        double eps_th = _eps(th, th_0, v_0, k);
        double num = v_0_2 * _sin2(th_0) - _v2(th, th_0, v_0, k) * _sin2(th);
        double denom = g * (2 + eps_th);
        return num / denom;
    }

    public static double _x(double th, double th_0, double v_0, double k, double g) {
        double v_0_2 = v_0 * v_0;
        double eps_th = _eps(th, th_0, v_0, k);
        double num = v_0_2 * sin(2 * th_0) - _v2(th, th_0, v_0, k) * sin(2 * th);
        double denom = 2 * g * (1 + eps_th);
        return num / denom;
    }

    private static double _ds(double z, double dx) {
        return 2 * z * dx;
    }

    private final static double epsilon = 0.00000001;

    private static double _dx(double v, double g) {
        return -2 * v * v / g;
    }

    private final static double descentParam = 0.1;

    private static double _ds2(double z, double dz, double dv, double dx2) {
        return 2 * (z * dx2 + dz * dv);
    }

    private static double _dv(double th, double v, double k) {
        return v * tan(th) + k * v * v * v / cos(th);
    }

    private static double _newton(double Th, double Th_0, double V_0, double Xt, double Yt, double G, double K) {
        double X = _x(Th, Th_0, V_0, K, G);
        double Y = _y(Th, Th_0, V_0, K, G);
        double Sx = X - Xt;
        double Sy = Y - Yt;
        double V = _v(Th, Th_0, V_0, K);
        double Dv = _dv(Th, V, K);
        double Dx = -V * V / G;
        double Dx2 = -2 * V * Dv / G;
        double Z = Sx + Sy * tan(Th);
        double Dz = (Dx + Sy) / _cos2(Th);
        double Ds = _ds(Z, Dx);
        double Ds2 = _ds2(Z, Dz, Dx, Dx2);
        return Ds / Ds2;
    }

    public static double minDist(double th_0, double v_0, double k, double g, double xt, double yt, double inc) {
        // Analytical approximation is only accurate when y = 0 or small changes in theta so use approximations as steps for quadrature
        double th_min = 0;
        double x = 0;
        double y = 0;
        for (double i = th_0; i > -PI / 2 + inc; i -= inc) {
            double v = _v(i, th_0, v_0, k);
            x += _x(i + inc, i, v, k, g);
            y += _y(i + inc, i, v, k, g);
            double sy = y - yt, sx = x - xt;
            double z = sx + sy * tan(i);
            double dx = _dx(v, g);
            double ds = _ds(z, dx);
            if (ds < 0) {
                return _approxMin(i, v, x, y, xt, yt, k, g);
            }
        }
        return th_min;
    }

    private static double _approxMin(double th_0, double v_0, double x, double y, double xt, double yt, double k, double g) {
        xt -= x;
        yt -= y;
        double th = th_0;
        for (int i = 0; i < kNMinLoops; i++) {
            double dt = _newton(th, th_0, v_0, xt - x, yt - y, k, g);
            th -= dt;
        }

        x = _x(th_0, th_0, v_0, k, g);
        y = _y(th_0, th_0, v_0, k, g);
        return x * x + y * y;
    }

    private static double cost(double th_0, double v_0, double xt, double yt) {
        return minDist(th_0, v_0, kProjectileConstant, kGravity, xt, yt, kQuadInc);
    }

    private static double[] findMin(double th_0, double v_0, double maxVelocity, double xt, double yt) {
        for (int i = 0; i < kNMinLoops; i++) {
            double m = cost(th_0, v_0, xt, yt);
            double Dv = (m - cost(th_0, v_0 + epsilon, xt, yt)) / epsilon;
            double Dt = (m - cost(th_0 + epsilon, v_0, xt, yt)) / epsilon;
            th_0 -= tanh(Dv) * descentParam;
            if (v_0 < maxVelocity) {
                v_0 -= tanh(Dt);
            } else {
                v_0 = maxVelocity;
            }
        }
        return new double[]{th_0, v_0};
    }

    public static double[] getOptimumParams(double xt, double yt) {
        double th_0 = atan2(yt, xt);
        double v_0 = kShooterTopVelocity - 3;
        return findMin(th_0, v_0, kShooterTopVelocity, xt, yt);
    }

  public static double getAngle(double xt) {
    return kHoodA * xt + kHoodB;
  }

  public static double polynomialInterpolation(double d1) {
    double d2 = d1*d1;
    return d2*kPolyA + d1*kPolyB + kPolyC;
  }
}