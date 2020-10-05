package us.ihmc.robotics.physics;

import static us.ihmc.robotics.physics.ContactImpulseTools.negateMult;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class ContactImpulseRandomTools
{
   public static DMatrixRMaj nextSquareFullRank(Random random)
   {
      return nextSquareFullRank(random, 1.0e-3, 10.0);
   }

   public static DMatrixRMaj nextSquareFullRank(Random random, double singularValueMin, double singularValueMax)
   {
      return nextSquareFullRank(random, singularValueMin, singularValueMax, 0.25);
   }

   public static DMatrixRMaj nextSquareFullRank(Random random, double singularValueMin, double singularValueMax, double probabilityNegativeSingularValue)
   {
      double[] singularValues = new double[3];

      for (int j = 0; j < 3; j++)
      {
         singularValues[j] = EuclidCoreRandomTools.nextDouble(random, singularValueMin, singularValueMax);
         if (random.nextDouble() < probabilityNegativeSingularValue)
            singularValues[j] = -singularValues[j];
      }
      DMatrixRMaj M_inv = RandomMatrices_DDRM.singular(3, 3, random, singularValues);
      return M_inv;
   }

   public static DMatrixRMaj nextPositiveDefiniteMatrix(Random random, double singularValueMin, double singularValueMax)
   {
      DMatrixRMaj M = RandomMatrices_DDRM.diagonal(3, singularValueMin, singularValueMax, random);
      DMatrixRMaj P = nextSquareFullRank(random);
      DMatrixRMaj Pinv = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.invert(P, Pinv);
      DMatrixRMaj PM = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(P, M, PM);
      DMatrixRMaj PMPinv = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(PM, Pinv, PMPinv);
      return PMPinv;
   }

   public static DMatrixRMaj nextPositiveDefiniteSymmetricMatrix(Random random, double min, double max)
   {
      return RandomMatrices_DDRM.symmetricWithEigenvalues(3, random, RandomNumbers.nextDoubleArray(random, 3, min, max));
   }

   public static DMatrixRMaj nextSlippingClosingVelocity(Random random, DMatrixRMaj M_inv, double mu)
   {
      DMatrixRMaj lambda_v_0 = new DMatrixRMaj(3, 1);
      lambda_v_0.set(2, EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 10.0));

      double minFrictionImpulse = mu * lambda_v_0.get(2);
      double frictionImpulse = EuclidCoreRandomTools.nextDouble(random, 1.01, 10.0) * minFrictionImpulse;

      double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      lambda_v_0.set(0, frictionImpulse * Math.cos(theta));
      lambda_v_0.set(1, frictionImpulse * Math.sin(theta));
      DMatrixRMaj c = negateMult(M_inv, lambda_v_0);

      if (c.get(2) > 0.0)
      {
         for (double alpha = theta; alpha < theta + 2.0 * Math.PI; alpha += 0.1 * Math.PI)
         {
            lambda_v_0.set(0, frictionImpulse * Math.cos(alpha));
            lambda_v_0.set(1, frictionImpulse * Math.sin(alpha));
            c = negateMult(M_inv, lambda_v_0);
            if (c.get(2) < 0.0)
               return c;
         }
         return null;
      }

      return c;
   }
}
