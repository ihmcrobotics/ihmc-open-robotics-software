package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;

import georegression.struct.se.Se3_F64;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class GeoregressionConversionTools
{
   public static void setVecmathTransformFromGeoregressionTransform(RigidBodyTransform vecmathTransform, Se3_F64 georegressionTransform)
   {
      double[] values = new double[4 * 4];
      for (int r = 0; r < 3; r++)
      {
         for (int c = 0; c < 3; c++)
         {
            values[r * 4 + c] = georegressionTransform.getR().get(r, c);
         }
      }

      for (int r = 0; r < 3; r++)
      {
         values[r * 4 + 3] = georegressionTransform.getT().getIndex(r);
      }

      for (int c = 0; c < 3; c++)
      {
         values[3 * 4 + c] = 0.0;
      }

      values[3 * 4 + 3] = 1;
      vecmathTransform.set(values);
   }

   public static void setGeoregressionTransformFromVecmath(RigidBodyTransform vecmathTransform, Se3_F64 georegressionTransform)
   {
      double[] m1 = new double[16];
      vecmathTransform.get(m1);
      double[][] rot = new double[][]
      {
         {m1[0], m1[1], m1[2]}, {m1[4], m1[5], m1[6]}, {m1[8], m1[9], m1[10]}
      };
      DenseMatrix64F denseMatrix64F = new DenseMatrix64F(rot);
      georegressionTransform.setRotation(denseMatrix64F);
      georegressionTransform.setTranslation(m1[3], m1[7], m1[11]);
   }

}
