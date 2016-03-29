package us.ihmc.robotics.numericalMethods;

public class QuarticEquationSolver implements java.io.Serializable
{
   private static final long serialVersionUID = -256535947296433934L;

   public QuarticEquationSolver()
   {
   }

   public void solveCubicEquation(double p3, double p2, double p1, double p0, double[] solution_real, double[] solution_imag)
   {
      // Solves p3x^3 + p2x^2 + p1x + p0 = 0 and returns the 3 solutions in solution[]
      if (Math.abs(p3) < EQN_EPS)
      {
         solveQuadraticEquation(p2, p1, p0, solution_real, solution_imag);
         solution_real[2] = 0.0;
         solution_imag[2] = 0.0;
      }

      else
         solveCubicEquation(p2 / p3, p1 / p3, p0 / p3, solution_real, solution_imag);

   }


   public void solveCubicEquation(double A, double B, double C, double[] solution_real, double[] solution_imag)
   {
      solution_imag[0] = solution_imag[1] = solution_imag[2] = 0.0;
      int num_real_solutions = solveCubicEquationReal(A, B, C, solution_real);

//    System.out.println("Found " + num_real_solutions + " real solutions");

      if (num_real_solutions > 1)
         return;

      double y_real = solution_real[0];

      this.solveQuadraticEquation(1.0, A + y_real, A * y_real + y_real * y_real + B, solution_real, solution_imag);
      solution_real[2] = y_real;
   }


   private static final double EQN_EPS = 1e-9;

   private int solveCubicEquationReal(double A, double B, double C, double[] s)
   {
      int i, num;
      double sub;

      // double  A, B, C;
      double sq_A, p, q;
      double cb_p, D;


      /*
       *   substitute x = y - A/3 to eliminate quadric term:
       *   x^3 +px + q = 0
       */

      sq_A = A * A;
      p = 1.0 / 3 * (-1.0 / 3 * sq_A + B);
      q = 1.0 / 2 * (2.0 / 27 * A * sq_A - 1.0 / 3 * A * B + C);

      /* use Cardano's formula */

      cb_p = p * p * p;
      D = q * q + cb_p;

      if (Math.abs(D) < EQN_EPS)
      {
         if (Math.abs(q) < EQN_EPS)    /* one triple solution */
         {
            s[0] = 0;
            num = 1;
         }
         else    /* one single and one double solution */
         {
            double u = cbrt(-q);

//          System.out.println("Taking cube root of " + (-q) + ".  Answer is " + u);
            s[0] = 2 * u;
            s[1] = -u;
            num = 2;
         }
      }
      else if (D < 0)    /* Casus irreducibilis: three real solutions */
      {
         double phi = 1.0 / 3 * Math.acos(-q / Math.sqrt(-cb_p));
         double t = 2 * Math.sqrt(-p);

         s[0] = t * Math.cos(phi);
         s[1] = -t * Math.cos(phi + Math.PI / 3.0);
         s[2] = -t * Math.cos(phi - Math.PI / 3.0);
         num = 3;
      }
      else    /* one real solution */
      {
         double sqrt_D = Math.sqrt(D);
         double u = cbrt(sqrt_D - q);

//       System.out.println("Taking cube root of " + (sqrt_D - q) + ".  Answer is " + u);
         double v = -cbrt(sqrt_D + q);

//       System.out.println("Taking negative cube root of " + (sqrt_D + q) + ".  Answer is " + -v);

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


   private static final double ONE_THIRD = 1.0 / 3.0;

   private double cbrt(double x)
   {
      if (x >= 0.0)
         return Math.pow(x, ONE_THIRD);
      else
         return -Math.pow(-x, ONE_THIRD);


   }



   @SuppressWarnings("unused")
   private double[] cube_root_t_real = new double[3], cube_root_u_real = new double[3];
   @SuppressWarnings("unused")
   private double[] cube_root_t_imag = new double[3], cube_root_u_imag = new double[3];

   @SuppressWarnings("unused")
   private void solveCubicEquation9(double e, double f, double g, double[] solution_real, double[] solution_imag)
   {
      // Solves x^3 + ex^2 + fx + g = 0
      // Substitute x = y-e/3 to get y^3 + py + q = 0:

      // System.out.println("Solving Cubic Equation:  e = " + e + ", f = " + f + ", g = " + g);
      double p = f - e * e / 3.0;
      double q = -f * e / 3.0 + g + 2.0 / 27.0 * e * e * e;

      // System.out.println("p = " + p + ", q = " + q);

      double kernel = 4.0 * p * p * p + 27.0 * q * q;
      if (kernel < 0.0)
         kernel = -kernel;

      double t = -q / 2.0 + Math.sqrt(kernel) / (6.0 * Math.sqrt(3.0));
      double u = q / 2.0 + Math.sqrt(kernel) / (6.0 * Math.sqrt(3.0));

      double y1;

      if ((t >= 0.0) && (u >= 0.0))
         y1 = Math.pow(t, 1.0 / 3.0) - Math.pow(u, 1.0 / 3.0);    // The first, real solution
      else if ((t >= 0.0) && (u < 0.0))
         y1 = Math.pow(t, 1.0 / 3.0) - Math.pow(-u, 1.0 / 3.0);
      else if ((t < 0.0) && (u >= 0.0))
         y1 = Math.pow(-t, 1.0 / 3.0) - Math.pow(u, 1.0 / 3.0);
      else
         y1 = Math.pow(-t, 1.0 / 3.0) - Math.pow(-u, 1.0 / 3.0);

      // System.out.println("t: " + t + ", u: " + u + ", y1: " + y1);

      solveQuadraticEquation(1.0, y1, p + y1 * y1, solution_real, solution_imag);
      solution_real[0] -= e / 3.0;
      solution_real[1] -= e / 3.0;
      solution_real[2] = y1 - e / 3.0;
      solution_imag[2] = 0.0;

   }


   public void solveCubicEquation2(double e, double f, double g, double[] solution_real, double[] solution_imag)
   {
      // From Graphic Gems:

      double Q = (e * e - 3.0 * f) / 9.0;
      double R = (2.0 * e * e * e - 9.0 * e * f + 27.0 * g) / 54.0;
      double Qcubed = Q * Q * Q;
      double d = Qcubed - R * R;

      /* Three real roots */
      if (d >= 0.0)
      {
//       System.out.println("Three Real Roots");

         double theta = Math.acos(R / Math.sqrt(Qcubed));
         double sqrtQ = Math.sqrt(Q);
         solution_real[0] = -2.0 * sqrtQ * Math.cos(theta / 3.0) - e / 3.0;
         solution_real[1] = -2.0 * sqrtQ * Math.cos((theta + 2.0 * Math.PI) / 3.0) - e / 3.0;
         solution_real[2] = -2.0 * sqrtQ * Math.cos((theta + 4.0 * Math.PI) / 3.0) - e / 3.0;

         solution_imag[0] = solution_imag[1] = solution_imag[2] = 0.0;


      }

      /* One real root */
      else
      {
//       System.out.println("One Real Root");

//       System.out.println("R = " + R);
//       System.out.println("R^1/3:  " + Math.pow(Math.abs(R), 1.0 / 3.0));
         double ee = Math.sqrt(-d) + Math.pow(Math.abs(R), 1.0 / 3.0);
         solution_real[0] = (ee + Q / ee) - e / 3.0;
         if (R > 0.0)
            solution_real[0] = -solution_real[0];
         solution_imag[0] = 0.0;

         // Solve for the complex roots by solving the remaining quadratic equation:



      }




   }

   private double[] temp_real = new double[4], temp_imag = new double[4], temp2_real = new double[4], temp2_imag = new double[4];


   public void solveQuarticEquation(double p4, double p3, double p2, double p1, double p0, double[] solution_real, double[] solution_imag)
   {
      // Solves p4x^4 + p3x^3 + p2x^2 + p1x + p0 = 0 and returns the 4 solutions in solutions[].

      if (Math.abs(p4) < EQN_EPS)
      {
         solveCubicEquation(p3, p2, p1, p0, solution_real, solution_imag);
         solution_real[3] = 0.0;
         solution_imag[3] = 0.0;
      }

      else
         solveQuarticEquation(p3 / p4, p2 / p4, p1 / p4, p0 / p4, solution_real, solution_imag);
   }


   public void solveQuarticEquation(double a, double b, double c, double d, double[] solution_real, double[] solution_imag)
   {
      // Solves x^4 + ax^3 + bx^2 + cx + d = 0 and returns the 4 solutions in solutions[].
      // Following algorithm description in Dr. Math Faq.

      // If d=0, factor out x=0 and return the solution to the cubic:
      if (Math.abs(d) < EQN_EPS)
      {
         // System.out.println("D is 0.0 in solveQuarticEquation!!!!");
         solveCubicEquation(a, b, c, solution_real, solution_imag);
         solution_real[3] = 0.0;
         solution_imag[3] = 0.0;

         return;
      }

      // Substitute x=y-a/4, expand, and simplify to get y^4 + 0.0y^3 + ey^2 + fy + g = 0;

      double e = b - 3.0 * a * a / 8.0;
      double f = c + a * a * a / 8.0 - a * b / 2.0;
      double g = d - 3.0 * a * a * a * a / 256.0 + a * a * b / 16.0 - a * c / 4.0;

//    System.out.println("e: " + e + ", f: " + f + ", g: " + g);

      // Two special cases.  If g is zero, then have a cubic again:
      if (Math.abs(g) < EQN_EPS)
      {
         solveCubicEquation(1.0, e, f, solution_real, solution_imag);
         solution_real[0] -= a / 4.0;
         solution_real[1] -= a / 4.0;
         solution_real[2] -= a / 4.0;
         solution_real[3] = 0.0;
         solution_imag[3] = 0.0;

         return;
      }

      // If f = 0, then have a quadratic in y^2:  y^2 + ey^2 + g = 0
      if (Math.abs(f) < EQN_EPS)
      {
         solveQuadraticEquation(1.0, e, g, temp_real, temp_imag);

         // Compute the square roots of these to get the 4 solutions:

         solveSquareRoot(temp_real[0], temp_imag[0], solution_real, solution_imag);
         solution_real[2] = solution_real[0];
         solution_real[3] = solution_real[1];
         solution_imag[2] = solution_imag[0];
         solution_imag[3] = solution_imag[1];
         solveSquareRoot(temp_real[1], temp_imag[1], solution_real, solution_imag);

         return;

      }

      // Otherwise we have the general case with d,f,g all non-zero:
      // Euler solution method:

      solveCubicEquation(e / 2.0, (e * e - 4.0 * g) / 16.0, -f * f / 64.0, temp_real, temp_imag);

      solveSquareRoot(temp_real[0], temp_imag[0], temp2_real, temp2_imag);
      double p_real = temp2_real[0], p_imag = temp2_imag[0];
      solveSquareRoot(temp_real[1], temp_imag[1], temp2_real, temp2_imag);
      double q_real = temp2_real[0], q_imag = temp2_imag[0];

      // r = 1/(pq) * (-f/8)
      double pq_real = p_real * q_real - p_imag * q_imag;
      double pq_imag = p_real * q_imag + p_imag * q_real;

      double a2_b2 = pq_real * pq_real + pq_imag * pq_imag;
      pq_real = pq_real / a2_b2;
      pq_imag = -pq_imag / a2_b2;

      double r_real = -8.0 / f * pq_real;
      double r_imag = -8.0 / f * pq_imag;


      solution_real[0] = p_real + q_real + r_real - a / 4.0;
      solution_imag[0] = p_imag + q_imag + r_imag;
      solution_real[1] = p_real - q_real - r_real - a / 4.0;
      solution_imag[1] = p_imag - q_imag - r_imag;
      solution_real[2] = -p_real + q_real - r_real - a / 4.0;
      solution_imag[2] = -p_imag + q_imag - r_imag;
      solution_real[3] = -p_real - q_real + r_real - a / 4.0;
      solution_imag[3] = -p_imag - q_imag + r_imag;

      throw new RuntimeException("This doesn't seem to work. Fix and get the test cases passing...");
   }


   @SuppressWarnings("unused")
   private void solveCubeRoot(double x_real, double x_imag, double[] y_real, double[] y_imag)
   {
      // Return y = cuberoot(x) where x,y are complex:
      double r = Math.sqrt(x_real * x_real + x_imag * x_imag);
      double theta = Math.atan2(x_imag, x_real);

      double r2 = Math.pow(r, 1.0 / 3.0);
      double theta2_1 = theta / 3.0, theta2_2 = theta / 3.0 + 2.0 * Math.PI / 3.0, theta2_3 = theta / 3.0 + 4.0 * Math.PI / 3.0;

      y_real[0] = r2 * Math.cos(theta2_1);
      y_imag[0] = r2 * Math.sin(theta2_1);
      y_real[1] = r2 * Math.cos(theta2_2);
      y_imag[1] = r2 * Math.sin(theta2_2);
      y_real[2] = r2 * Math.cos(theta2_3);
      y_imag[2] = r2 * Math.sin(theta2_3);

   }

   private void solveSquareRoot(double x_real, double x_imag, double[] y_real, double[] y_imag)
   {
      // Return y = sqrt(x) where x,y are complex:
      double r = Math.sqrt(x_real * x_real + x_imag * x_imag);
      double theta = Math.atan2(x_imag, x_real);

      double r2 = Math.sqrt(r);
      double theta2_1 = theta / 2.0, theta2_2 = theta / 2.0 + Math.PI;

      y_real[0] = r2 * Math.cos(theta2_1);
      y_imag[0] = r2 * Math.sin(theta2_1);
      y_real[1] = r2 * Math.cos(theta2_2);
      y_imag[1] = r2 * Math.sin(theta2_2);


   }


   public void solveQuadraticEquation(double a, double b, double c, double[] x_real, double[] x_imag)
   {
      double two_a = 2.0 * a;
      x_imag[0] = x_imag[1] = 0.0;
      x_real[0] = x_real[1] = -b / two_a;
      double b2_4ac = b * b - 4.0 * a * c;
      double sqrt_b2_4ac;

      if (b2_4ac >= 0.0)
      {
         sqrt_b2_4ac = Math.sqrt(b2_4ac);
         x_real[0] += sqrt_b2_4ac / two_a;
         x_real[1] -= sqrt_b2_4ac / two_a;

         return;
      }

      else
      {
         sqrt_b2_4ac = Math.sqrt(-b2_4ac);
         x_imag[0] = sqrt_b2_4ac / two_a;
         x_imag[1] = -sqrt_b2_4ac / two_a;
      }
   }


}
