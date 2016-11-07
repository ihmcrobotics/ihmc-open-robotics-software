package us.ihmc.robotics.numericalMethods;


/*
 *
 * Is copyrighted, but can be used in any project, non-commercial and commercial.
 * http://www.realtimerendering.com/resources/GraphicsGems/
 *
 *
*  From Roots3And4.c in Graphics Gems.
*
*  Utility functions to find cubic and quartic roots,
*  coefficients are passed like this:
*
*      c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3 + c[4]*x^4 = 0
*
*  The functions return the number of non-complex roots and
*  put the values into the s array.
*
*  Author:         Jochen Schwarze (schwarze@isa.de)
*
*  Jan 26, 1990    Version for Graphics Gems
*  Oct 11, 1990    Fixed sign problem for negative q's in SolveQuartic
*                  (reported by Mark Podlipec),
*                  Old-style function definitions,
*                  IsZero() as a macro
*  Nov 23, 1990    Some systems do not declare acos() and cbrt() in
*                  <math.h>, though the functions exist in the library.
*                  If large coefficients are used, EQN_EPS should be
*                  reduced considerably (e.g. to 1E-30), results will be
*                  correct but multiple roots might be reported more
*                  than once.
*
*  Nov 5, 2001     Converted to Java by Jerry Pratt
 */

public class QuarticRootFinder implements java.io.Serializable
{
   private static final long serialVersionUID = 1492976164809974715L;
   private static final double EQN_EPS = 1e-9;

   public QuarticRootFinder()
   {
   }


   private static final double ONE_THIRD = 1.0 / 3.0;

   private double cubeRoot(double x)
   {
      if (x >= 0.0)
         return Math.pow(x, ONE_THIRD);
      else
         return -Math.pow(-x, ONE_THIRD);
   }

   private boolean IsZero(double x)
   {
      return ((x > -EQN_EPS) && (x < EQN_EPS));
   }


   public int SolveQuadric(double[] c, double[] s)    // c[3], s[2]
   {
      double p, q, D;

      /* normal form: x^2 + px + q = 0 */

      p = c[1] / (2.0 * c[2]);
      q = c[0] / c[2];

      D = p * p - q;

      if (IsZero(D))
      {
         s[0] = -p;

         return 1;
      }
      else if (D < 0.0)
      {
         return 0;
      }
      else    // if (D > 0.0)  Don't need this, it has to be D > 0.0
      {
         double sqrt_D = Math.sqrt(D);

         s[0] = sqrt_D - p;
         s[1] = -sqrt_D - p;

         return 2;
      }
   }

   private double[] cubic_overflow_coeffs = new double[3];

   public int SolveCubic(double[] c, double[] s)    // c[4], s[3]
   {
      int i, num;
      double sub;
      double A, B, C;
      double sq_A, p, q;
      double cb_p, D;

      /* normal form: x^3 + Ax^2 + Bx + C = 0 */

      if (IsZero(c[3]))    // +++JEP if c[3] is zero, then just a quadratic equation:
      {
         cubic_overflow_coeffs[0] = c[0];
         cubic_overflow_coeffs[1] = c[1];
         cubic_overflow_coeffs[2] = c[2];

         return SolveQuadric(cubic_overflow_coeffs, s);
      }

      A = c[2] / c[3];
      B = c[1] / c[3];
      C = c[0] / c[3];

      /*
       *   substitute x = y - A/3 to eliminate quadric term:
       *   x^3 +px + q = 0
       */

      sq_A = A * A;
      p = 1.0 / 3.0 * (-1.0 / 3.0 * sq_A + B);
      q = 1.0 / 2.0 * (2.0 / 27.0 * A * sq_A - 1.0 / 3.0 * A * B + C);

      /* use Cardano's formula */

      cb_p = p * p * p;
      D = q * q + cb_p;

      if (IsZero(D))
      {
         if (IsZero(q))    /* one triple solution */
         {
            s[0] = 0.0;
            num = 1;
         }
         else    /* one single and one double solution */
         {
            double u = cubeRoot(-q);
            s[0] = 2.0 * u;
            s[1] = -u;
            num = 2;
         }
      }
      else if (D < 0.0)    /* Casus irreducibilis: three real solutions */
      {
         double phi = 1.0 / 3.0 * Math.acos(-q / Math.sqrt(-cb_p));
         double t = 2.0 * Math.sqrt(-p);

         s[0] = t * Math.cos(phi);
         s[1] = -t * Math.cos(phi + Math.PI / 3.0);
         s[2] = -t * Math.cos(phi - Math.PI / 3.0);
         num = 3;
      }
      else    /* one real solution */
      {
         double sqrt_D = Math.sqrt(D);
         double u = cubeRoot(sqrt_D - q);
         double v = -cubeRoot(sqrt_D + q);

         s[0] = u + v;
         num = 1;
      }

      /* resubstitute */

      sub = 1.0 / 3.0 * A;

      for (i = 0; i < num; ++i)
      {
         s[i] -= sub;
      }

      return num;
   }


   private double[] coeffs = new double[4], s_temp = new double[2];
   private double[] quartic_overflow_coeffs = new double[4];

   public int SolveQuartic(double[] c, double[] s)    // c[5], s[4]
   {
      // double  coeffs[ 4 ];
      double z, u, v, sub;
      double A, B, C, D;
      double sq_A, p, q, r;
      int i, num;

      /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */

      if (IsZero(c[4]))    // +++ JEP.  If c[4] is zero, then zero is the root you're looking for!
      {
         quartic_overflow_coeffs[0] = c[0];
         quartic_overflow_coeffs[1] = c[1];
         quartic_overflow_coeffs[2] = c[2];
         quartic_overflow_coeffs[3] = c[3];

         return SolveCubic(quartic_overflow_coeffs, s);
      }

      A = c[3] / c[4];
      B = c[2] / c[4];
      C = c[1] / c[4];
      D = c[0] / c[4];

      /*
       *   substitute x = y - A/4 to eliminate cubic term:
       *   x^4 + px^2 + qx + r = 0
       */

      sq_A = A * A;
      p = -3.0 / 8.0 * sq_A + B;
      q = 1.0 / 8.0 * sq_A * A - 1.0 / 2.0 * A * B + C;
      r = -3.0 / 256.0 * sq_A * sq_A + 1.0 / 16.0 * sq_A * B - 1.0 / 4.0 * A * C + D;

      if (IsZero(r))
      {
         /* no absolute term: y(y^3 + py + q) = 0 */

         coeffs[0] = q;
         coeffs[1] = p;
         coeffs[2] = 0.0;
         coeffs[3] = 1.0;

         num = SolveCubic(coeffs, s);

         s[num++] = 0.0;
      }
      else
      {
         /* solve the resolvent cubic ... */

         coeffs[0] = 1.0 / 2.0 * r * p - 1.0 / 8.0 * q * q;
         coeffs[1] = -r;
         coeffs[2] = -1.0 / 2.0 * p;
         coeffs[3] = 1.0;

         SolveCubic(coeffs, s);

         /* ... and take the one real solution ... */

         z = s[0];

         /* ... to build two quadric equations */

         u = z * z - r;
         v = 2 * z - p;

         if (IsZero(u))
            u = 0.0;
         else if (u > 0.0)
            u = Math.sqrt(u);
         else
            return 0;

         if (IsZero(v))
            v = 0.0;
         else if (v > 0.0)
            v = Math.sqrt(v);
         else
            return 0;

         coeffs[0] = z - u;
         coeffs[1] = (q < 0.0) ? -v : v;
         coeffs[2] = 1.0;

         num = SolveQuadric(coeffs, s);

         coeffs[0] = z + u;
         coeffs[1] = (q < 0.0) ? v : -v;
         coeffs[2] = 1.0;

         // num += SolveQuadric(coeffs, s + num);  JEP+++ Can't do this in Java, need to pass another array...
         int num2 = SolveQuadric(coeffs, s_temp);

         for (int j = 0; j < num2; j++)
         {
            s[j + num] = s_temp[j];
         }

         num = num + num2;

         // / +++JEP That should do the trick!
      }

      /* resubstitute */

      sub = 1.0 / 4.0 * A;

      for (i = 0; i < num; ++i)
      {
         s[i] -= sub;
      }

      return num;
   }

}

