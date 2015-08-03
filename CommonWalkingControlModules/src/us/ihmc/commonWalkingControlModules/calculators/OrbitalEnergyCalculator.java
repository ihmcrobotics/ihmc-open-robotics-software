package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoUtilities.math.trajectories.YoPolynomial;


public class OrbitalEnergyCalculator
{
   public static double computeOrbitalEnergy(YoPolynomial spline, double g, double x, double xd)
   {
      spline.compute(x);
      double f = spline.getPosition();
      double fPrime = spline.getVelocity();
      double h = f - fPrime * x;
      double integralTerm = computeIntegralTerm(spline, x);
      double eOrbit = 0.5 * MathTools.square(xd * h) + g * MathTools.square(x) * f - 3.0 * g * integralTerm;

      return eOrbit;
   }

   public static double computeOrbitalEnergyConstantHeight(double z, double g, double x, double xd)
   {
      return 0.5 * MathTools.square(xd) - g / (2.0 * z) * MathTools.square(xd);
   }
   
   private static double computeIntegralTerm(YoPolynomial spline, double x)
   {
      double integral = 0.0;
      double xPower = x;
      for (int i = 0; i < spline.getNumberOfCoefficients(); i++)
      {
         xPower *= x;
         integral += 1.0 / (i + 2.0) * spline.getCoefficient(i) * xPower;
      }

      return integral;
   }

   public static void computeParabolicCoefficientsUsingOrbitalEnergy(YoPolynomial spline, double orbitalEnergyDesired, double g, double x0, double xd0,
           double z0, double dzdx0)
   {
      double a0 = (-4 * orbitalEnergyDesired + (g * Math.pow(x0, 2) + 2 * Math.pow(xd0, 2) * (dzdx0 * x0 - z0)) * (dzdx0 * x0 - z0)) / (g * Math.pow(x0, 2));
      double a1 = (8 * orbitalEnergyDesired - dzdx0 * Math.pow(x0, 2) * (3 * g * x0 + 4 * dzdx0 * Math.pow(xd0, 2))
                   + 4 * x0 * (g * x0 + 2 * dzdx0 * Math.pow(xd0, 2)) * z0 - 4 * Math.pow(xd0, 2) * Math.pow(z0, 2)) / (g * Math.pow(x0, 3));
      double a2 = (2 * (-2 * orbitalEnergyDesired + (g * Math.pow(x0, 2) + Math.pow(xd0, 2) * (dzdx0 * x0 - z0)) * (dzdx0 * x0 - z0))) / (g * Math.pow(x0, 4));
      double[] a = new double[] {a0, a1, a2, 0.0, 0.0};
      spline.setDirectly(a);
   }

   public static void computeParabolicCoefficientsUsingZFinal(YoPolynomial spline, double g, double x0, double xd0, double xf, double z0, double dzdx0,
           double zf)
   {
      double a0 = (xf * (dzdx0 * x0 * (x0 - xf) + (-2 * x0 + xf) * z0) + Math.pow(x0, 2) * zf) / Math.pow(x0 - xf, 2);
      double a1 = (dzdx0 * (-Math.pow(x0, 2) + Math.pow(xf, 2)) + 2 * x0 * (z0 - zf)) / Math.pow(x0 - xf, 2);
      double a2 = (dzdx0 * (x0 - xf) - z0 + zf) / Math.pow(x0 - xf, 2);
      double[] a = new double[] {a0, a1, a2, 0.0, 0.0};
      spline.setDirectly(a);
   }

   public static void computeCubicCoefficientsGivenZf(YoPolynomial spline, double orbitalEnergyDesired, double g, double x0, double xd0, double xf, double z0,
           double dzdx0, double zf)
   {
      double a0 =
         (20 * orbitalEnergyDesired * Math.pow(x0 - xf, 2) * xf
          + xf * (-(dzdx0 * Math.pow(x0, 2) * (g * x0 * (3 * x0 - 5 * xf) + 10 * dzdx0 * Math.pow(xd0, 2) * (x0 - xf)) * (x0 - xf))
             + x0 * (20 * dzdx0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) + g * x0 * (Math.pow(x0, 2) - 8 * x0 * xf + 5 * Math.pow(xf, 2))) * z0
             - 10 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * Math.pow(z0, 2)) + 2 * g * Math.pow(x0, 5) * zf) / (g * Math.pow(x0, 2) * (2 * x0 - 5 * xf) * Math.pow(x0 - xf, 2));
      double a1 =
         (-20 * orbitalEnergyDesired * Math.pow(x0 - xf, 2) * (x0 + 2 * xf)
          + 10 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (x0 + 2 * xf)
          + 3 * dzdx0 * g * Math.pow(x0, 3) * (Math.pow(x0, 3) - 6 * x0 * Math.pow(xf, 2) + 5 * Math.pow(xf, 3))
          - 20 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (x0 + 2 * xf) * z0
          + z0 * (-(g * Math.pow(x0, 2) * (Math.pow(x0, 3) - 30 * x0 * Math.pow(xf, 2) + 20 * Math.pow(xf, 3)))
             + 10 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (x0 + 2 * xf) * z0) - 9 * g * Math.pow(x0, 5) * zf) / (g * Math.pow(x0, 3) * (2 * x0 - 5 * xf) * Math.pow(x0 - xf, 2));
      double a2 = (2 * (10 * orbitalEnergyDesired * Math.pow(x0 - xf, 2) * (2 * x0 + xf)
                        - 5 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (2 * x0 + xf)
                        + dzdx0 * g * Math.pow(x0, 3) * (-4 * Math.pow(x0, 3) + 9 * Math.pow(x0, 2) * xf - 5 * Math.pow(xf, 3))
                        + 10 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (2 * x0 + xf) * z0
                        + z0 * (g * Math.pow(x0, 2) * (4 * Math.pow(x0, 3) - 15 * Math.pow(x0, 2) * xf + 5 * Math.pow(xf, 3))
                                - 5 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * (2 * x0 + xf) * z0) + 6 * g * Math.pow(x0, 5) * zf)) / (g * Math.pow(x0, 4)
                                   * (2 * x0 - 5 * xf) * Math.pow(x0 - xf, 2));
      double a3 = (-5 * (4 * orbitalEnergyDesired * Math.pow(x0 - xf, 2) - 2 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2)
                         + dzdx0 * x0 * (x0 - xf) * (-(g * Math.pow(x0, 2) * (x0 - 2 * xf)) + 4 * Math.pow(xd0, 2) * (x0 - xf) * z0)
                         + z0 * (g * Math.pow(x0, 2) * (Math.pow(x0, 2) - 4 * x0 * xf + 2 * Math.pow(xf, 2))
                                 - 2 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 2) * z0) + g * Math.pow(x0, 4) * zf)) / (g * Math.pow(x0, 4) * (2 * x0 - 5 * xf)
                                    * Math.pow(x0 - xf, 2));

      double[] a = new double[] {a0, a1, a2, a3, 0.0};
      spline.setDirectly(a);
   }

   public static void computeCubicCoefficientsGivenDzdxf(YoPolynomial spline, double orbitalEnergyDesired, double g, double x0, double xd0, double xf,
           double z0, double dzdx0, double dzdxf)
   {
      double a0 =
         (-20 * orbitalEnergyDesired * (x0 - 3 * xf) * (x0 - xf)
          + Math.pow(x0, 2)
            * (-2 * dzdxf * g * Math.pow(x0, 3) + 10 * Math.pow(dzdx0, 2) * Math.pow(xd0, 2) * (x0 - 3 * xf) * (x0 - xf)
               + dzdx0 * g * x0 * (3 * Math.pow(x0, 2) - 16 * x0 * xf + 15 * Math.pow(xf, 2))) - x0 * (g * x0 * (x0 - 15 * xf) + 20 * dzdx0 * Math.pow(xd0, 2) * (x0 - 3 * xf)) * (x0 - xf) * z0 + 10 * Math.pow(xd0, 2) * (x0 - 3 * xf) * (x0 - xf) * Math.pow(z0, 2)) / (3. * g * Math.pow(x0, 2) * (3 * x0 - 5 * xf) * (x0 - xf));

      double a1 = (3 * dzdxf * g * Math.pow(x0, 5)
                   + xf * (dzdx0 * Math.pow(x0, 2) * (3 * g * x0 * (4 * x0 - 5 * xf) + 20 * dzdx0 * Math.pow(xd0, 2) * (x0 - xf))
                           + 40 * orbitalEnergyDesired * (-x0 + xf) - 20 * x0 * (g * x0 + 2 * dzdx0 * Math.pow(xd0, 2)) * (x0 - xf) * z0
                           + 20 * Math.pow(xd0, 2) * (x0 - xf) * Math.pow(z0, 2))) / (g * Math.pow(x0, 3) * (3 * x0 - 5 * xf) * (x0 - xf));

      double a2 = (2 * (-(Math.pow(x0, 4) * (3 * dzdx0 * g * x0 + 2 * dzdxf * g * x0 + 5 * Math.pow(dzdx0, 2) * Math.pow(xd0, 2)))
                        + 5 * dzdx0 * Math.pow(x0, 2) * (g * x0 + dzdx0 * Math.pow(xd0, 2)) * Math.pow(xf, 2)
                        + 10 * orbitalEnergyDesired * (x0 - xf) * (x0 + xf) + 5 * x0 * (g * x0 + 2 * dzdx0 * Math.pow(xd0, 2)) * (x0 - xf) * (x0 + xf) * z0
                        + 5 * Math.pow(xd0, 2) * (-Math.pow(x0, 2) + Math.pow(xf, 2)) * Math.pow(z0, 2))) / (g * Math.pow(x0, 4) * (3 * x0 - 5 * xf)
                           * (x0 - xf));

      double a3 =
         (5 * (Math.pow(x0, 2) * (dzdxf * g * Math.pow(x0, 2) + dzdx0 * g * x0 * (3 * x0 - 4 * xf) + 4 * Math.pow(dzdx0, 2) * Math.pow(xd0, 2) * (x0 - xf))
               + 8 * orbitalEnergyDesired * (-x0 + xf) - 4 * x0 * (g * x0 + 2 * dzdx0 * Math.pow(xd0, 2)) * (x0 - xf) * z0
               + 4 * Math.pow(xd0, 2) * (x0 - xf) * Math.pow(z0, 2))) / (3. * g * Math.pow(x0, 4) * (3 * x0 - 5 * xf) * (x0 - xf));

      double[] a = new double[] {a0, a1, a2, a3, 0.0};
      spline.setDirectly(a);
   }

   public static void computeQuarticCoefficients(YoPolynomial spline, double orbitalEnergyDesired, double g, double x0, double xd0, double xf, double z0,
           double dzdx0, double zf, double dzdxf)
   {
      double a0 = (-(dzdxf * g * Math.pow(x0, 5) * (x0 - 2 * xf) * (x0 - xf) * xf) + 2 * dzdx0 * g * Math.pow(x0, 6) * Math.pow(xf, 2)
                   + 10 * Math.pow(dzdx0, 2) * Math.pow(x0, 5) * Math.pow(xd0, 2) * Math.pow(xf, 2)
                   - 20 * orbitalEnergyDesired * Math.pow(x0 - xf, 3) * Math.pow(xf, 2) - 8 * dzdx0 * g * Math.pow(x0, 5) * Math.pow(xf, 3)
                   - 30 * Math.pow(dzdx0, 2) * Math.pow(x0, 4) * Math.pow(xd0, 2) * Math.pow(xf, 3) + 11 * dzdx0 * g * Math.pow(x0, 4) * Math.pow(xf, 4)
                   + 30 * Math.pow(dzdx0, 2) * Math.pow(x0, 3) * Math.pow(xd0, 2) * Math.pow(xf, 4) - 5 * dzdx0 * g * Math.pow(x0, 3) * Math.pow(xf, 5)
                   - 10 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(xf, 5) + 2 * g * Math.pow(x0, 5) * Math.pow(xf, 2) * z0
                   - 20 * dzdx0 * Math.pow(x0, 4) * Math.pow(xd0, 2) * Math.pow(xf, 2) * z0 + 2 * g * Math.pow(x0, 4) * Math.pow(xf, 3) * z0
                   + 60 * dzdx0 * Math.pow(x0, 3) * Math.pow(xd0, 2) * Math.pow(xf, 3) * z0 - 11 * g * Math.pow(x0, 3) * Math.pow(xf, 4) * z0
                   - 60 * dzdx0 * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(xf, 4) * z0 + 5 * g * Math.pow(x0, 2) * Math.pow(xf, 5) * z0
                   + 20 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(xf, 5) * z0 + 10 * Math.pow(x0, 3) * Math.pow(xd0, 2) * Math.pow(xf, 2) * Math.pow(z0, 2)
                   - 30 * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(xf, 3) * Math.pow(z0, 2)
                   + 30 * x0 * Math.pow(xd0, 2) * Math.pow(xf, 4) * Math.pow(z0, 2) - 10 * Math.pow(xd0, 2) * Math.pow(xf, 5) * Math.pow(z0, 2)
                   + g * Math.pow(x0, 5) * (Math.pow(x0, 2) - 7 * x0 * xf + 8 * Math.pow(xf, 2)) * zf) / (g * Math.pow(x0, 2) * Math.pow(x0 - xf, 3)
                     * (Math.pow(x0, 2) - 4 * x0 * xf + 5 * Math.pow(xf, 2)));

      double a1 =
         (dzdxf * g * Math.pow(x0, 5) * (x0 - xf) * (Math.pow(x0, 2) + 2 * x0 * xf - 9 * Math.pow(xf, 2))
          - xf * (-40 * orbitalEnergyDesired * Math.pow(x0 - xf, 3) * (x0 + xf)
             + 20 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (x0 + xf)
             + dzdx0 * g * Math.pow(x0, 3) * (x0 - xf) * (4 * Math.pow(x0, 3) - 7 * Math.pow(x0, 2) * xf - 6 * x0 * Math.pow(xf, 2) + 15 * Math.pow(xf, 3))
             - 40 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (x0 + xf) * z0
             + 4 * z0
               * (g * Math.pow(x0, 2) * (Math.pow(x0, 4) + Math.pow(x0, 3) * xf - 10 * x0 * Math.pow(xf, 3) + 5 * Math.pow(xf, 4))
                  + 5 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (x0 + xf) * z0) + 12 * g * Math.pow(x0, 5) * (-2 * x0 + 3 * xf) * zf)) / (g * Math.pow(x0, 3)
                     * Math.pow(x0 - xf, 3) * (Math.pow(x0, 2) - 4 * x0 * xf + 5 * Math.pow(xf, 2)));

      double a2 = (2 * (-2 * dzdxf * g * Math.pow(x0, 8) + 4 * dzdxf * g * Math.pow(x0, 7) * xf + 4 * dzdxf * g * Math.pow(x0, 6) * Math.pow(xf, 2)
                     - 6 * dzdxf * g * Math.pow(x0, 5) * Math.pow(xf, 3)
                     - 10 * orbitalEnergyDesired * Math.pow(x0 - xf, 3) * (Math.pow(x0, 2) + 4 * x0 * xf + Math.pow(xf, 2))
                     + 5 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (Math.pow(x0, 2) + 4 * x0 * xf + Math.pow(xf, 2))
                     + dzdx0 * g * Math.pow(x0, 3) * (x0 - xf)
                       * (Math.pow(x0, 4) + 2 * Math.pow(x0, 3) * xf - 12 * Math.pow(x0, 2) * Math.pow(xf, 2) + 10 * x0 * Math.pow(xf, 3)
                          + 5 * Math.pow(xf, 4)) + g * Math.pow(x0, 7) * z0 + g * Math.pow(x0, 6) * xf * z0 + 16 * g * Math.pow(x0, 5) * Math.pow(xf, 2) * z0
                             - 40 * g * Math.pow(x0, 4) * Math.pow(xf, 3) * z0 + 5 * g * Math.pow(x0, 3) * Math.pow(xf, 4) * z0
                             + 5 * g * Math.pow(x0, 2) * Math.pow(xf, 5) * z0
                             - 10 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (Math.pow(x0, 2) + 4 * x0 * xf + Math.pow(xf, 2)) * z0
                             + 5 * Math.pow(x0, 5) * Math.pow(xd0, 2) * Math.pow(z0, 2) + 5 * Math.pow(x0, 4) * Math.pow(xd0, 2) * xf * Math.pow(z0, 2)
                             - 40 * Math.pow(x0, 3) * Math.pow(xd0, 2) * Math.pow(xf, 2) * Math.pow(z0, 2)
                             + 40 * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(xf, 3) * Math.pow(z0, 2)
                             - 5 * x0 * Math.pow(xd0, 2) * Math.pow(xf, 4) * Math.pow(z0, 2) - 5 * Math.pow(xd0, 2) * Math.pow(xf, 5) * Math.pow(z0, 2)
                             - 6 * g * Math.pow(x0, 5) * (Math.pow(x0, 2) + x0 * xf - 4 * Math.pow(xf, 2)) * zf)) / (g * Math.pow(x0, 4) * Math.pow(x0 - xf, 3)
                                * (Math.pow(x0, 2) - 4 * x0 * xf + 5 * Math.pow(xf, 2)));

      double a3 = (5 * (dzdxf * g * Math.pow(x0, 7) - 3 * dzdxf * g * Math.pow(x0, 6) * xf + dzdxf * g * Math.pow(x0, 5) * Math.pow(xf, 2)
                        + dzdxf * g * Math.pow(x0, 4) * Math.pow(xf, 3) + 8 * orbitalEnergyDesired * Math.pow(x0 - xf, 3) * (x0 + xf)
                        - 4 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (x0 + xf)
                        - dzdx0 * g * Math.pow(x0, 3) * (x0 - xf) * (Math.pow(x0, 3) - 2 * Math.pow(x0, 2) * xf - x0 * Math.pow(xf, 2) + 4 * Math.pow(xf, 3))
                        - 4 * g * Math.pow(x0, 5) * xf * z0 + 4 * g * Math.pow(x0, 4) * Math.pow(xf, 2) * z0 + 8 * g * Math.pow(x0, 3) * Math.pow(xf, 3) * z0
                        - 4 * g * Math.pow(x0, 2) * Math.pow(xf, 4) * z0 + 8 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * (x0 + xf) * z0
                        - 4 * Math.pow(x0, 4) * Math.pow(xd0, 2) * Math.pow(z0, 2) + 8 * Math.pow(x0, 3) * Math.pow(xd0, 2) * xf * Math.pow(z0, 2)
                        - 8 * x0 * Math.pow(xd0, 2) * Math.pow(xf, 3) * Math.pow(z0, 2) + 4 * Math.pow(xd0, 2) * Math.pow(xf, 4) * Math.pow(z0, 2)
                        + 4 * g * Math.pow(x0, 4) * (Math.pow(x0, 2) - x0 * xf - Math.pow(xf, 2)) * zf)) / (g * Math.pow(x0, 4) * Math.pow(x0 - xf, 3)
                           * (Math.pow(x0, 2) - 4 * x0 * xf + 5 * Math.pow(xf, 2)));

      double a4 = (-2 * dzdxf * g * Math.pow(x0, 6) - 20 * orbitalEnergyDesired * Math.pow(x0 - xf, 3)
                   + 10 * Math.pow(dzdx0, 2) * Math.pow(x0, 2) * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) + 7 * dzdxf * g * Math.pow(x0, 5) * xf
                   - 5 * dzdxf * g * Math.pow(x0, 4) * Math.pow(xf, 2)
                   + dzdx0 * g * Math.pow(x0, 3) * (x0 - xf) * (3 * Math.pow(x0, 2) - 10 * x0 * xf + 10 * Math.pow(xf, 2)) - g * Math.pow(x0, 5) * z0
                   - 20 * dzdx0 * x0 * Math.pow(xd0, 2) * Math.pow(x0 - xf, 3) * z0 + 15 * g * Math.pow(x0, 4) * xf * z0
                   - 30 * g * Math.pow(x0, 3) * Math.pow(xf, 2) * z0 + 10 * g * Math.pow(x0, 2) * Math.pow(xf, 3) * z0
                   + 10 * Math.pow(x0, 3) * Math.pow(xd0, 2) * Math.pow(z0, 2) - 30 * Math.pow(x0, 2) * Math.pow(xd0, 2) * xf * Math.pow(z0, 2)
                   + 30 * x0 * Math.pow(xd0, 2) * Math.pow(xf, 2) * Math.pow(z0, 2) - 10 * Math.pow(xd0, 2) * Math.pow(xf, 3) * Math.pow(z0, 2)
                   + 3 * g * Math.pow(x0, 4) * (-3 * x0 + 5 * xf) * zf) / (g * Math.pow(x0, 4) * Math.pow(x0 - xf, 3)
                     * (Math.pow(x0, 2) - 4 * x0 * xf + 5 * Math.pow(xf, 2)));

      double[] a = new double[] {a0, a1, a2, a3, a4};
      spline.setDirectly(a);
   }
}
