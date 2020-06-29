package us.ihmc.robotics.physics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.NormOps_DDRM;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * This class implements the math introduced in Section III of <i>"Per-Contact Iteration Method for
 * Solving Contact Dynamics"</i>.
 *
 * @author Sylvain Bertrand
 */
public class ContactImpulseTools
{
   public static DMatrixRMaj cross(DMatrixRMaj v1, DMatrixRMaj v2)
   {
      DMatrixRMaj cross = new DMatrixRMaj(3, 1);
      cross(v1, v2, cross);
      return cross;
   }

   public static void cross(DMatrixRMaj v1, DMatrixRMaj v2, DMatrixRMaj cross)
   {
      if (!MatrixFeatures_DDRM.isVector(v1) || v1.getNumElements() != 3)
         throw new IllegalArgumentException("Improper argument");
      if (!MatrixFeatures_DDRM.isVector(v2) || v2.getNumElements() != 3)
         throw new IllegalArgumentException("Improper argument");

      cross.reshape(3, 1);

      double x = v1.get(1) * v2.get(2) - v1.get(2) * v2.get(1);
      double y = v1.get(2) * v2.get(0) - v1.get(0) * v2.get(2);
      double z = v1.get(0) * v2.get(1) - v1.get(1) * v2.get(0);
      cross.set(0, x);
      cross.set(1, y);
      cross.set(2, z);
   }

   public static DMatrixRMaj invert(DMatrixRMaj M)
   {
      DMatrixRMaj M_inv = new DMatrixRMaj(M.getNumCols(), M.getNumRows());
      CommonOps_DDRM.invert(M, M_inv);
      return M_inv;
   }

   /**
    * Computes: <tt>x<sup>T</sup>Hx</tt> with {@code x} being vector such that the result of this
    * operation is a scalar.
    */
   public static double multQuad(DMatrixRMaj x, DMatrixRMaj H)
   {
      if (!MatrixFeatures_DDRM.isVector(x))
         throw new IllegalArgumentException("x is not a vector.");
      if (x.getNumRows() != H.getNumRows())
         throw new IllegalArgumentException("x and H are not compatible.");
      if (!MatrixFeatures_DDRM.isSquare(H))
         throw new IllegalArgumentException("H is not square.");
      double result = 0.0;

      for (int row = 0; row < H.getNumRows(); row++)
      {
         double rowValue = 0.0;

         for (int col = 0; col < H.getNumCols(); col++)
         {
            rowValue += H.unsafe_get(row, col) * x.get(col);
         }

         result += x.get(row) * rowValue;
      }

      return result;
   }

   /**
    * <pre>
    * return = a + b * c
    * </pre>
    */
   public static DMatrixRMaj addMult(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c)
   {
      DMatrixRMaj d = new DMatrixRMaj(b.getNumRows(), c.getNumCols());
      addMult(a, b, c, d);
      return d;
   }

   /**
    * <pre>
    * d = a + b * c
    * </pre>
    */
   public static void addMult(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c, DMatrixRMaj d)
   {
      CommonOps_DDRM.mult(b, c, d);
      CommonOps_DDRM.addEquals(d, a);
   }

   /**
    * <pre>
    * return = -a * b
    * </pre>
    */
   public static DMatrixRMaj negateMult(DMatrixRMaj a, DMatrixRMaj b)
   {
      DMatrixRMaj c = new DMatrixRMaj(a.getNumRows(), b.getNumCols());
      negateMult(a, b, c);
      return c;
   }

   /**
    * <pre>
    * c = -a * b
    * </pre>
    */
   public static void negateMult(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c)
   {
      CommonOps_DDRM.mult(a, b, c);
      CommonOps_DDRM.changeSign(c);
   }

   public static double computeR(double mu, double theta, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      return computeR(mu, Math.cos(theta), Math.sin(theta), M_inv, c);
   }

   public static double computeR(double mu, double cosTheta, double sinTheta, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      // Equation (14) in the paper
      //                                -c_z
      // r = -------------------------------------------------------------
      //     M_inv_zz / mu + M_inv_zx * cos(theta) + M_inv_zy * sin(theta)
      double r = M_inv.get(8) / mu + M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta;
      r = -c.get(2) / r;
      return r;
   }

   public static double computeLambdaZ(double r, double theta, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      return computeLambdaZ(r, Math.cos(theta), Math.sin(theta), M_inv, c);
   }

   public static double computeLambdaZ(double r, double cosTheta, double sinTheta, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      // Equation (13) in the paper
      // lambda_z = (-c_z - M_inv_zx * r * cos(theta) - M_inv_zy * r * sin(theta)) / M_inv_zz
      return -(c.get(2) + r * (M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta)) / M_inv.get(8);
   }

   public static DMatrixRMaj computePostImpulseVelocity(DMatrixRMaj c, DMatrixRMaj M_inv, DMatrixRMaj lambda)
   {
      return addMult(c, M_inv, lambda);
   }

   public static double computeE1(DMatrixRMaj v, DMatrixRMaj M)
   {
      // E = v^T M v
      return 0.5 * multQuad(v, M);
   }

   public static double computeE2(DMatrixRMaj M_inv, DMatrixRMaj c, DMatrixRMaj lambda)
   {
      // E = 0.5 * lambda^T M^-1 lambda + lambda^T c
      return 0.5 * multQuad(lambda, M_inv) + CommonOps_DDRM.dot(lambda, c);
   }

   public static double computeE3(DMatrixRMaj M, DMatrixRMaj M_inv, DMatrixRMaj c, DMatrixRMaj lambda)
   {
      // E = 0.5 * lambda^T M^-1 lambda + lambda^T c + 0.5 * c^T M c
      return computeE2(M_inv, c, lambda) + 0.5 * multQuad(c, M);
   }

   public static DMatrixRMaj nablaH1(DMatrixRMaj M_inv)
   { // Introduced in Section III.A. Careful there's a typo in the paper where M is used instead of its inverse.
      DMatrixRMaj nablaH1 = new DMatrixRMaj(3, 1);
      nablaH1.set(0, M_inv.get(2, 0));
      nablaH1.set(1, M_inv.get(2, 1));
      nablaH1.set(2, M_inv.get(2, 2));
      return nablaH1;
   }

   public static DMatrixRMaj nablaH2(DMatrixRMaj lambda, double mu)
   {
      DMatrixRMaj nablaH2 = new DMatrixRMaj(3, 1);
      nablaH2.set(0, 2.0 * lambda.get(0));
      nablaH2.set(1, 2.0 * lambda.get(1));
      nablaH2.set(2, -2.0 * mu * mu * lambda.get(2));
      return nablaH2;
   }

   public static DMatrixRMaj eta(DMatrixRMaj M_inv, DMatrixRMaj lambda, double mu)
   {
      DMatrixRMaj nablaH1 = nablaH1(M_inv);
      DMatrixRMaj nablaH2 = nablaH2(lambda, mu);
      DMatrixRMaj eta = cross(nablaH1, nablaH2);
      NormOps_DDRM.normalizeF(eta);
      return eta;
   }

   public static double computeProjectedGradient(double mu, DMatrixRMaj M_inv, DMatrixRMaj c, DMatrixRMaj lambda)
   {
      return computeProjectedGradient(mu, M_inv, c, EuclidCoreTools.norm(lambda.get(0), lambda.get(1)), Math.atan2(lambda.get(1), lambda.get(0)));
   }

   public static double computeProjectedGradient(double mu, DMatrixRMaj M_inv, DMatrixRMaj c, double theta)
   {
      return computeProjectedGradient(mu, M_inv, c, computeR(mu, theta, M_inv, c), theta);
   }

   public static double computeProjectedGradient(double mu, DMatrixRMaj M_inv, DMatrixRMaj c, double r, double theta)
   {
      return computeProjectedGradient(mu, M_inv, c, r, Math.cos(theta), Math.sin(theta));
   }

   public static double computeProjectedGradient(double mu, DMatrixRMaj M_inv, DMatrixRMaj c, double r, double cosTheta, double sinTheta)
   {
      double nablaH1_x = M_inv.get(6);
      double nablaH1_y = M_inv.get(7);
      double nablaH1_z = M_inv.get(8);

      double nablaH2_x = cosTheta;
      double nablaH2_y = sinTheta;
      double nablaH2_z = -mu;

      // Equation (10) in the paper
      double eta_x = nablaH1_y * nablaH2_z - nablaH1_z * nablaH2_y;
      double eta_y = nablaH1_z * nablaH2_x - nablaH1_x * nablaH2_z;
      double eta_z = nablaH1_x * nablaH2_y - nablaH1_y * nablaH2_x;

      // Equation (15) in the paper, we actually use the left equality only so skipping equation (16).
      double r_inv = 1.0 / r;
      double c_x = c.get(0) * r_inv;
      double c_y = c.get(1) * r_inv;
      double c_z = c.get(2) * r_inv;

      double lambda_z_prime = (M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta + c_z) / M_inv.get(8);

      double dE_dLambda_0 = M_inv.get(0) * cosTheta + M_inv.get(1) * sinTheta + c_x - M_inv.get(2) * lambda_z_prime;
      double dE_dLambda_1 = M_inv.get(3) * cosTheta + M_inv.get(4) * sinTheta + c_y - M_inv.get(5) * lambda_z_prime;
      double dE_dLambda_2 = M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta + c_z - M_inv.get(8) * lambda_z_prime;

      return dE_dLambda_0 * eta_x + dE_dLambda_1 * eta_y + dE_dLambda_2 * eta_z;
   }

   public static double computeProjectedGradientInefficient(DMatrixRMaj M_inv, DMatrixRMaj lambda, DMatrixRMaj c, double mu)
   {
      DMatrixRMaj gradient = addMult(c, M_inv, lambda);
      DMatrixRMaj eta = eta(M_inv, lambda, mu);
      return CommonOps_DDRM.dot(gradient, eta);
   }

   public static double computeEThetaNumericalDerivative(double theta, double dtheta, double mu, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      double costMinus = computeE2(M_inv, c, computeLambda(theta - 0.5 * dtheta, mu, M_inv, c));
      double costPlus = computeE2(M_inv, c, computeLambda(theta + 0.5 * dtheta, mu, M_inv, c));
      return (costPlus - costMinus) / dtheta;
   }

   public static DMatrixRMaj computeLambda(double theta, double mu, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      return computeLambda(theta, computeR(mu, theta, M_inv, c), mu, M_inv, c);
   }

   public static DMatrixRMaj computeLambda(double theta, double r, double mu, DMatrixRMaj M_inv, DMatrixRMaj c)
   {
      DMatrixRMaj lambda = new DMatrixRMaj(3, 1);
      lambda.set(0, r * Math.cos(theta));
      lambda.set(1, r * Math.sin(theta));
      lambda.set(2, computeLambdaZ(r, theta, M_inv, c));
      return lambda;
   }

   public static boolean isInsideFrictionCone(double mu, DMatrixRMaj lambda)
   {
      return isInsideFrictionCone(mu, lambda, 0.0);
   }

   public static boolean isInsideFrictionCone(double mu, DMatrixRMaj lambda, double epsilon)
   {
      return isInsideFrictionCone(mu, lambda.get(0), lambda.get(1), lambda.get(2), epsilon);
   }

   public static boolean isInsideFrictionCone(double mu, double lambda_x, double lambda_y, double lambda_z, double epsilon)
   {
      return EuclidCoreTools.square(mu * lambda_z) + epsilon >= EuclidCoreTools.normSquared(lambda_x, lambda_y);
   }

   public static boolean lineOfSightTest(double mu, DMatrixRMaj lambda, DMatrixRMaj lambda_v_0)
   {
      double theta = Math.atan2(lambda.get(1), lambda.get(0));
      return lineOfSightTest(mu, EuclidCoreTools.norm(lambda.get(0), lambda.get(1)), Math.cos(theta), Math.sin(theta), lambda_v_0);
   }

   public static boolean lineOfSightTest(double mu, double r, double cosTheta, double sinTheta, DMatrixRMaj lambda_v_0)
   {
      // Simplified line-of-sight for NableH2 . (lambda_v_0 - lambda) < 0
      return cosTheta * lambda_v_0.get(0) + sinTheta * lambda_v_0.get(1) - mu * lambda_v_0.get(2) > 0.0;
   }

   public static DMatrixRMaj computeSlipLambda(double beta1, double beta2, double beta3, double gamma, double mu, DMatrixRMaj M_inv, DMatrixRMaj lambda_v_0,
                                               DMatrixRMaj c, boolean verbose)
   {
      Vector3D lambdaOpt = new Vector3D();
      computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, lambdaOpt, verbose);
      DMatrixRMaj lambdaOptMatrix = new DMatrixRMaj(3, 1);
      lambdaOpt.get(lambdaOptMatrix);
      return lambdaOptMatrix;
   }

   /**
    * This method implements the bisection method used to find the optimal impulse that minimize the
    * tangential velocity while canceling the normal velocity.
    * <p>
    * This algorithm will fail if the contact is opening or sticking. The z-component of the given data
    * is assumed to be the normal component of the contact.
    * </p>
    * <p>
    * Before starting the bisection, a lower and upper bounds have to be determined. The beta
    * parameters are used for this initial phase. No value were suggested in the paper.
    * </p>
    * 
    * @param beta1                algorithm's parameter used for the initial guessing.
    * @param beta2                algorithm's parameter used for stepping backward.
    * @param beta3                algorithm's parameter used for stepping forward.
    * @param gamma                algorithm's termination parameter.
    * @param mu                   the coefficient of friction.
    * @param M_inv                the 3-by-3 inverse of the apparent inertia matrix. Not modified.
    * @param lambda_v_0           the 3-by-1 impulse that fully cancels the contact velocity while
    *                             ignoring the cone of friction. Not modified.
    * @param c                    the 3-by-1 contact velocity. Not modified.
    * @param contactImpulseToPack output of this algorithm: the impulse that satisfies the cone of
    *                             friction, minimize the tangential velocity, and cancel the normal
    *                             velocity. Modified.
    * @param verbose              useful for debugging when this algorithm is acting up.
    */
   public static void computeSlipLambda(double beta1, double beta2, double beta3, double gamma, double mu, DMatrixRMaj M_inv, DMatrixRMaj lambda_v_0,
                                        DMatrixRMaj c, Tuple3DBasics contactImpulseToPack, boolean verbose)
   {
      // Initial guess using lambda_v_0
      double thetaLo0 = Math.atan2(lambda_v_0.get(1), lambda_v_0.get(0));
      double cosTheta = Math.cos(thetaLo0);
      double sinTheta = Math.sin(thetaLo0);
      double rLo0 = computeR(mu, cosTheta, sinTheta, M_inv, c);
      double gradient_v_0 = computeProjectedGradient(mu, M_inv, c, rLo0, cosTheta, sinTheta);
      double alpha = -beta1 * Math.signum(gradient_v_0);

      double thetaHi0;
      double rHi0;

      int iteration = 0;

      if (verbose)
         System.out.println("Initial stepping, theta 0: " + thetaLo0);

      while (true)
      {
         thetaHi0 = thetaLo0;
         rHi0 = rLo0;

         thetaLo0 += alpha;
         if (verbose)
            System.out.println("Stepping theta " + thetaLo0);
         cosTheta = Math.cos(thetaLo0);
         sinTheta = Math.sin(thetaLo0);
         rLo0 = computeR(mu, cosTheta, sinTheta, M_inv, c);

         if (rLo0 < 0.0)
         {
            alpha = beta2 * alpha;
            if (verbose)
               System.out.println("r negative: " + rLo0 + ", alpha " + alpha);
            thetaLo0 = thetaHi0;
            rLo0 = rHi0;
         }
         else
         {
            if (computeLambdaZ(rLo0, cosTheta, sinTheta, M_inv, c) < 0.0)
            {
               System.err.println(toStringForUnitTest(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c));
               throw new IllegalStateException("Malformed impule");
            }

            if (!lineOfSightTest(mu, rLo0, cosTheta, sinTheta, lambda_v_0))
            {
               alpha *= beta2;
               thetaLo0 = thetaHi0;
               rLo0 = rHi0;
               if (verbose)
                  System.out.println("out of line-of-sight, alpha " + alpha);
            }
            else
            {
               double gradientLo0 = computeProjectedGradient(mu, M_inv, c, rLo0, cosTheta, sinTheta);

               if (verbose)
                  System.out.println("Gradient " + gradientLo0 + " (D0=" + gradient_v_0 + ")");

               if (gradientLo0 * gradient_v_0 > 0.0)
               {
                  alpha *= beta3;
                  if (verbose)
                     System.out.println("same sign gradient, alpha " + alpha);
               }
               else
               {
                  break;
               }
            }
         }

         iteration++;
         if (iteration > 1000)
         {
            System.err.println(toStringForUnitTest(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c));
            throw new IllegalStateException("Unable to converge during initialization of the bisection");
         }
      }

      double thetaLo = thetaLo0;
      double thetaHi = thetaHi0;

      double thetaMid;
      double cosThetaMid, sinThetaMid;
      double rMid;

      iteration = 0;

      while (true)
      {
         thetaMid = 0.5 * (thetaLo + thetaHi);
         cosThetaMid = Math.cos(thetaMid);
         sinThetaMid = Math.sin(thetaMid);
         rMid = computeR(mu, cosThetaMid, sinThetaMid, M_inv, c);

         double gradientMid = computeProjectedGradient(mu, M_inv, c, rMid, cosThetaMid, sinThetaMid);

         if (gradientMid * gradient_v_0 > 0.0)
         {
            thetaHi = thetaMid;
         }
         else
         {
            thetaLo = thetaMid;
         }

         if (Math.abs(thetaLo - thetaHi) < gamma)
            break;

         iteration++;
         if (iteration > 1000)
         {
            System.err.println(toStringForUnitTest(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c));
            throw new IllegalStateException("Unable to converge during bisection");
         }
      }

      double lambdaMid_x = rMid * cosThetaMid;
      double lambdaMid_y = rMid * sinThetaMid;
      double lambdaMid_z = computeLambdaZ(rMid, cosThetaMid, sinThetaMid, M_inv, c);
      contactImpulseToPack.set(lambdaMid_x, lambdaMid_y, lambdaMid_z);
   }

   public static String toStringForUnitTest(double beta1, double beta2, double beta3, double gamma, double mu, DMatrixRMaj M_inv, DMatrixRMaj lambda_v_0,
                                            DMatrixRMaj c)
   {
      return beta1 + ", " + beta2 + ", " + beta3 + ", " + gamma + ", " + mu + ", " + matrixToString(M_inv) + ", " + matrixToString(c);
   }

   private static String matrixToString(DMatrixRMaj m)
   {
      String ret = "new DMatrixRMaj(" + m.getNumRows() + ", " + m.getNumCols() + ", true";
      for (int i = 0; i < m.getNumElements(); i++)
         ret += ", " + m.get(i);
      ret += ")";
      return ret;
   }

   public static double cube(double value)
   {
      return value * value * value;
   }
}
