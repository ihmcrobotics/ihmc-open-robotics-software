package us.ihmc.robotics.physics;

import static us.ihmc.robotics.physics.ContactImpulseTools.negateMult;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class ContactImpulseRandomTools
{
   public static DenseMatrix64F nextSquareFullRank(Random random)
   {
      return nextSquareFullRank(random, 1.0e-3, 10.0);
   }

   public static DenseMatrix64F nextSquareFullRank(Random random, double singularValueMin, double singularValueMax)
   {
      return nextSquareFullRank(random, singularValueMin, singularValueMax, 0.25);
   }

   public static DenseMatrix64F nextSquareFullRank(Random random, double singularValueMin, double singularValueMax, double probabilityNegativeSingularValue)
   {
      double[] singularValues = new double[3];

      for (int j = 0; j < 3; j++)
      {
         singularValues[j] = EuclidCoreRandomTools.nextDouble(random, singularValueMin, singularValueMax);
         if (random.nextDouble() < probabilityNegativeSingularValue)
            singularValues[j] = -singularValues[j];
      }
      DenseMatrix64F M_inv = RandomMatrices.createSingularValues(3, 3, random, singularValues);
      return M_inv;
   }

   public static DenseMatrix64F nextPositiveDefiniteMatrix(Random random, double singularValueMin, double singularValueMax)
   {
      DenseMatrix64F M = RandomMatrices.createDiagonal(3, singularValueMin, singularValueMax, random);
      DenseMatrix64F P = nextSquareFullRank(random);
      DenseMatrix64F Pinv = new DenseMatrix64F(3, 3);
      CommonOps.invert(P, Pinv);
      DenseMatrix64F PM = new DenseMatrix64F(3, 3);
      CommonOps.mult(P, M, PM);
      DenseMatrix64F PMPinv = new DenseMatrix64F(3, 3);
      CommonOps.mult(PM, Pinv, PMPinv);
      return PMPinv;
   }

   public static DenseMatrix64F nextPositiveDefiniteSymmetricMatrix(Random random, double min, double max)
   {
      return RandomMatrices.createEigenvaluesSymm(3, random, RandomNumbers.nextDoubleArray(random, 3, min, max));
   }

   public static DenseMatrix64F nextSlippingClosingVelocity(Random random, DenseMatrix64F M_inv, double mu)
   {
      DenseMatrix64F lambda_v_0 = new DenseMatrix64F(3, 1);
      lambda_v_0.set(2, EuclidCoreRandomTools.nextDouble(random, 1.0e-2, 10.0));

      double minFrictionImpulse = mu * lambda_v_0.get(2);
      double frictionImpulse = EuclidCoreRandomTools.nextDouble(random, 1.01, 10.0) * minFrictionImpulse;

      double theta = EuclidCoreRandomTools.nextDouble(random, Math.PI);
      lambda_v_0.set(0, frictionImpulse * Math.cos(theta));
      lambda_v_0.set(1, frictionImpulse * Math.sin(theta));
      DenseMatrix64F c = negateMult(M_inv, lambda_v_0);

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
