package us.ihmc.robotics.physics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.NormOps;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class ContactImpulseTools
{
   public static DenseMatrix64F cross(DenseMatrix64F v1, DenseMatrix64F v2)
   {
      DenseMatrix64F cross = new DenseMatrix64F(3, 1);
      cross(v1, v2, cross);
      return cross;
   }

   public static void cross(DenseMatrix64F v1, DenseMatrix64F v2, DenseMatrix64F cross)
   {
      if (!MatrixFeatures.isVector(v1) || v1.getNumElements() != 3)
         throw new IllegalArgumentException("Improper argument");
      if (!MatrixFeatures.isVector(v2) || v2.getNumElements() != 3)
         throw new IllegalArgumentException("Improper argument");

      cross.reshape(3, 1);

      double x = v1.get(1) * v2.get(2) - v1.get(2) * v2.get(1);
      double y = v1.get(2) * v2.get(0) - v1.get(0) * v2.get(2);
      double z = v1.get(0) * v2.get(1) - v1.get(1) * v2.get(0);
      cross.set(0, x);
      cross.set(1, y);
      cross.set(2, z);
   }

   public static DenseMatrix64F invert(DenseMatrix64F M)
   {
      DenseMatrix64F M_inv = new DenseMatrix64F(M.getNumCols(), M.getNumRows());
      CommonOps.invert(M, M_inv);
      return M_inv;
   }

   public static double multQuad(DenseMatrix64F x, DenseMatrix64F H)
   {
      if (!MatrixFeatures.isVector(x))
         throw new IllegalArgumentException("x is not a vector.");
      if (x.getNumRows() != H.getNumRows())
         throw new IllegalArgumentException("x and H are not compatible.");
      if (!MatrixFeatures.isSquare(H))
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
   public static DenseMatrix64F addMult(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      DenseMatrix64F d = new DenseMatrix64F(b.getNumRows(), c.getNumCols());
      addMult(a, b, c, d);
      return d;
   }

   /**
    * <pre>
    * d = a + b * c
    * </pre>
    */
   public static void addMult(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F d)
   {
      CommonOps.mult(b, c, d);
      CommonOps.addEquals(d, a);
   }

   /**
    * <pre>
    * return = -a * b
    * </pre>
    */
   public static DenseMatrix64F negateMult(DenseMatrix64F a, DenseMatrix64F b)
   {
      DenseMatrix64F c = new DenseMatrix64F(a.getNumRows(), b.getNumCols());
      negateMult(a, b, c);
      return c;
   }

   /**
    * <pre>
    * c = -a * b
    * </pre>
    */
   public static void negateMult(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      CommonOps.mult(a, b, c);
      CommonOps.changeSign(c);
   }

   public static double computeR(double mu, double theta, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      return computeR(mu, Math.cos(theta), Math.sin(theta), M_inv, c);
   }

   public static double computeR(double mu, double cosTheta, double sinTheta, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      // Equation (14) in the paper
      //                                -c_z
      // r = -------------------------------------------------------------
      //     M_inv_zz / mu + M_inv_zx * cos(theta) + M_inv_zy * sin(theta)
      double r = M_inv.get(8) / mu + M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta;
      r = -c.get(2) / r;
      return r;
   }

   public static double computeLambdaZ(double r, double theta, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      return ContactImpulseTools.computeLambdaZ(r, Math.cos(theta), Math.sin(theta), M_inv, c);
   }

   public static double computeLambdaZ(double r, double cosTheta, double sinTheta, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      // Equation (13) in the paper
      // lambda_z = (-c_z - M_inv_zx * r * cos(theta) - M_inv_zy * r * sin(theta)) / M_inv_zz
      return -(c.get(2) + r * (M_inv.get(6) * cosTheta + M_inv.get(7) * sinTheta)) / M_inv.get(8);
   }

   public static DenseMatrix64F computePostImpulseVelocity(DenseMatrix64F c, DenseMatrix64F M_inv, DenseMatrix64F lambda)
   {
      return addMult(c, M_inv, lambda);
   }

   public static void computePostImpulseVelocity(DenseMatrix64F c, DenseMatrix64F M_inv, DenseMatrix64F lambda, DenseMatrix64F vToPack)
   {
      addMult(c, M_inv, lambda, vToPack);
   }

   public static double computeE1(DenseMatrix64F v, DenseMatrix64F M)
   {
      // E = v^T M v
      return 0.5 * multQuad(v, M);
   }

   public static double computeE2(DenseMatrix64F M_inv, DenseMatrix64F c, DenseMatrix64F lambda)
   {
      // E = 0.5 * lambda^T M^-1 lambda + lambda^T c
      return 0.5 * multQuad(lambda, M_inv) + CommonOps.dot(lambda, c);
   }

   public static double computeE3(DenseMatrix64F M, DenseMatrix64F M_inv, DenseMatrix64F c, DenseMatrix64F lambda)
   {
      // E = 0.5 * lambda^T M^-1 lambda + lambda^T c + 0.5 * c^T M c
      return computeE2(M_inv, c, lambda) + 0.5 * multQuad(c, M);
   }

   public static DenseMatrix64F nablaH1(DenseMatrix64F M_inv)
   { // Introduced in Section III.A. Careful there's a typo in the paper where M is used instead of its inverse.
      DenseMatrix64F nablaH1 = new DenseMatrix64F(3, 1);
      nablaH1.set(0, M_inv.get(2, 0));
      nablaH1.set(1, M_inv.get(2, 1));
      nablaH1.set(2, M_inv.get(2, 2));
      return nablaH1;
   }

   public static DenseMatrix64F nablaH2(DenseMatrix64F lambda, double mu)
   {
      DenseMatrix64F nablaH2 = new DenseMatrix64F(3, 1);
      nablaH2.set(0, 2.0 * lambda.get(0));
      nablaH2.set(1, 2.0 * lambda.get(1));
      nablaH2.set(2, -2.0 * mu * mu * lambda.get(2));
      return nablaH2;
   }

   public static DenseMatrix64F eta(DenseMatrix64F M_inv, DenseMatrix64F lambda, double mu)
   {
      DenseMatrix64F nablaH1 = nablaH1(M_inv);
      DenseMatrix64F nablaH2 = nablaH2(lambda, mu);
      DenseMatrix64F eta = cross(nablaH1, nablaH2);
      NormOps.normalizeF(eta);
      return eta;
   }

   public static double computeProjectedGradient(double mu, DenseMatrix64F M_inv, DenseMatrix64F c, DenseMatrix64F lambda)
   {
      return ContactImpulseTools.computeProjectedGradient(mu,
                                                          M_inv,
                                                          c,
                                                          EuclidCoreTools.norm(lambda.get(0), lambda.get(1)),
                                                          Math.atan2(lambda.get(1), lambda.get(0)));
   }

   public static double computeProjectedGradient(double mu, DenseMatrix64F M_inv, DenseMatrix64F c, double theta)
   {
      return computeProjectedGradient(mu, M_inv, c, computeR(mu, theta, M_inv, c), theta);
   }

   public static double computeProjectedGradient(double mu, DenseMatrix64F M_inv, DenseMatrix64F c, double r, double theta)
   {
      return computeProjectedGradient(mu, M_inv, c, r, Math.cos(theta), Math.sin(theta));
   }

   public static double computeProjectedGradient(double mu, DenseMatrix64F M_inv, DenseMatrix64F c, double r, double cosTheta, double sinTheta)
   {
      // Equation (15) in the paper
      double nablaH1_x = M_inv.get(6);
      double nablaH1_y = M_inv.get(7);
      double nablaH1_z = M_inv.get(8);

      double nablaH2_x = cosTheta;
      double nablaH2_y = sinTheta;
      double nablaH2_z = -mu;

      double eta_x = nablaH1_y * nablaH2_z - nablaH1_z * nablaH2_y;
      double eta_y = nablaH1_z * nablaH2_x - nablaH1_x * nablaH2_z;
      double eta_z = nablaH1_x * nablaH2_y - nablaH1_y * nablaH2_x;

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

   public static double computeProjectedGradientInefficient(DenseMatrix64F M_inv, DenseMatrix64F lambda, DenseMatrix64F c, double mu)
   {
      DenseMatrix64F gradient = ContactImpulseTools.addMult(c, M_inv, lambda);
      DenseMatrix64F eta = ContactImpulseTools.eta(M_inv, lambda, mu);
      return CommonOps.dot(gradient, eta);
   }

   public static double computeEThetaNumericalDerivative(double theta, double dtheta, double mu, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      double costMinus = ContactImpulseTools.computeE2(M_inv, c, computeLambda(theta - 0.5 * dtheta, mu, M_inv, c));
      double costPlus = ContactImpulseTools.computeE2(M_inv, c, computeLambda(theta + 0.5 * dtheta, mu, M_inv, c));
      return (costPlus - costMinus) / dtheta;
   }

   public static DenseMatrix64F computeLambda(double theta, double mu, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      return computeLambda(theta, ContactImpulseTools.computeR(mu, theta, M_inv, c), mu, M_inv, c);
   }

   public static DenseMatrix64F computeLambda(double theta, double r, double mu, DenseMatrix64F M_inv, DenseMatrix64F c)
   {
      DenseMatrix64F lambda = new DenseMatrix64F(3, 1);
      lambda.set(0, r * Math.cos(theta));
      lambda.set(1, r * Math.sin(theta));
      lambda.set(2, ContactImpulseTools.computeLambdaZ(r, theta, M_inv, c));
      return lambda;
   }

   public static boolean isInsideFrictionCone(double mu, DenseMatrix64F lambda)
   {
      return isInsideFrictionCone(mu, lambda, 0.0);
   }

   public static boolean isInsideFrictionCone(double mu, DenseMatrix64F lambda, double epsilon)
   {
      return isInsideFrictionCone(mu, lambda.get(0), lambda.get(1), lambda.get(2), epsilon);
   }

   public static boolean isInsideFrictionCone(double mu, double lambda_x, double lambda_y, double lambda_z)
   {
      return isInsideFrictionCone(mu, lambda_x, lambda_y, lambda_z, 0.0);
   }

   public static boolean isInsideFrictionCone(double mu, double lambda_x, double lambda_y, double lambda_z, double epsilon)
   {
      return EuclidCoreTools.square(mu * lambda_z) + epsilon >= EuclidCoreTools.normSquared(lambda_x, lambda_y);
   }

   public static boolean lineOfSightTest(double mu, DenseMatrix64F lambda, DenseMatrix64F lambda_v_0)
   {
      double theta = Math.atan2(lambda.get(1), lambda.get(0));
      return lineOfSightTest(mu, EuclidCoreTools.norm(lambda.get(0), lambda.get(1)), Math.cos(theta), Math.sin(theta), lambda_v_0);
   }

   public static boolean lineOfSightTest(double mu, double r, double cosTheta, double sinTheta, DenseMatrix64F lambda_v_0)
   {
      // Simplified line-of-sight for NableH2 . (lambda_v_0 - lambda) < 0
      return cosTheta * lambda_v_0.get(0) + sinTheta * lambda_v_0.get(1) - mu * lambda_v_0.get(2) > 0.0;
   }

   public static DenseMatrix64F computeSlipLambda(double beta1, double beta2, double beta3, double gamma, double mu, DenseMatrix64F M_inv,
                                                  DenseMatrix64F lambda_v_0, DenseMatrix64F c, boolean verbose)
   {
      Vector3D lambdaOpt = new Vector3D();
      computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_inv, lambda_v_0, c, lambdaOpt, verbose);
      DenseMatrix64F lambdaOptMatrix = new DenseMatrix64F(3, 1);
      lambdaOpt.get(lambdaOptMatrix);
      return lambdaOptMatrix;
   }

   public static boolean computeSlipLambda(double beta1, double beta2, double beta3, double gamma, double mu, DenseMatrix64F M_inv, DenseMatrix64F lambda_v_0,
                                           DenseMatrix64F c, Tuple3DBasics contactImpulseToPack, boolean verbose)
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
      return true;
   }

   public static String toStringForUnitTest(double beta1, double beta2, double beta3, double gamma, double mu, DenseMatrix64F M_inv, DenseMatrix64F lambda_v_0,
                                            DenseMatrix64F c)
   {
      return beta1 + ", " + beta2 + ", " + beta3 + ", " + gamma + ", " + mu + ", " + matrixToString(M_inv) + ", " + matrixToString(c);
   }

   private static String matrixToString(DenseMatrix64F m)
   {
      String ret = "new DenseMatrix64F(" + m.getNumRows() + ", " + m.getNumCols() + ", true";
      for (int i = 0; i < m.getNumElements(); i++)
         ret += ", " + m.get(i);
      ret += ")";
      return ret;
   }

   public static double cube(double value)
   {
      return value * value * value;
   }

   public static double polarGradient2(DenseMatrix64F M_inv, DenseMatrix64F c, double theta, double mu)
   { // Obtained by directly evaluating dE/dTheta
      double c_x = c.get(0);
      double c_y = c.get(1);
      double c_z = c.get(2);

      double cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);

      double Mxx = M_inv.get(0, 0);
      double Mxy = M_inv.get(0, 1);
      double Myy = M_inv.get(1, 1);
      double Mzx = M_inv.get(2, 0);
      double Mzy = M_inv.get(2, 1);
      double Mzz = M_inv.get(2, 2);

      double Mtheta = Mzx * cosTheta + Mzy * sinTheta;

      return -c_z * mu
            * (((-Mzy * c_x + Mzx * c_y) * Mtheta + (Mzy * (Mxx * cosTheta + Mxy * sinTheta) - Mzx * (Mxy * cosTheta + Myy * sinTheta)) * c_z) * mu * mu
                  + Mzz * (-(sinTheta * Mtheta + Mzy) * c_x + (cosTheta * Mtheta + Mzx) * c_y
                        + (cosTheta * sinTheta * Mxx - Mxy + 2 * sinTheta * sinTheta * Mxy - sinTheta * cosTheta * Myy) * c_z) * mu
                  + Mzz * ((-sinTheta * c_x + cosTheta * c_y) * Mzz + c_z * (Mzx * sinTheta - Mzy * cosTheta)))
            / cube(Mzz + Mtheta * mu);
   }

}
