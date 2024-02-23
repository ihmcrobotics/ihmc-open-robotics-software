package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.Random;

public class SavitzkyGolayOnlineOrientationFilter3DTest
{

   public static final double ITERATIONS = 10000;

   @Test
   public void test_expSO3()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3DReadOnly vector = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         Matrix3D actual = new Matrix3D();
         SavitzkyGolayOnlineOrientationFilter3D.expSO3(vector, actual);
         Matrix3D expected = naive_expSO3(vector);
         EuclidCoreTestTools.assertEquals(expected, actual, 1e-12);
      }
   }

   @Test
   public void test_dexpSO3()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3DReadOnly vector = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         Matrix3D actual = new Matrix3D();
         SavitzkyGolayOnlineOrientationFilter3D.dexpSO3(vector, actual);
         Matrix3D expected = naive_dexpSO3(vector);
         EuclidCoreTestTools.assertEquals(expected, actual, 1e-12);
      }
   }

   @Test
   public void test_ddexpSO3()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3DReadOnly x = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         Vector3DReadOnly z = EuclidCoreRandomTools.nextVector3D(random, 10.0);
         Matrix3D actual = new Matrix3D();
         SavitzkyGolayOnlineOrientationFilter3D.ddexpSO3(x, z, actual);
         Matrix3D expected = naive_ddexpSO3(x, z);
         EuclidCoreTestTools.assertEquals(expected, actual, 1e-12);
      }
   }

   public static Matrix3D naive_expSO3(Vector3DReadOnly a)
   {
      // From the MATLAB code:
      //phi = norm(a);
      //if phi ~=0
      //   res = eye(3) + sin(phi)/phi*hat(a) + (1-cos(phi))/phi^2*hat(a)*hat(a);
      //else
      //   res = eye(3);
      double phi = a.norm();
      Matrix3D res = new Matrix3D();
      res.setIdentity();
      if (phi != 0)
      {
         Matrix3D aHat = hat(a);
         aHat.scale(Math.sin(phi) / phi);
         res.add(aHat);
         aHat = hat(a);
         aHat.multiply(aHat);
         aHat.scale((1 - Math.cos(phi)) / (phi * phi));
         res.add(aHat);
      }
      return res;
   }

   public static Matrix3D naive_dexpSO3(Vector3DReadOnly a)
   {
      // The MATLAB code:
      //      phi = norm(a);
      //      ahat = hat(a);
      //      beta = sin(phi/2)^2/((phi/2)^2);
      //      alpha = sin(phi)/phi;
      //      res = eye(3)+0.5*beta*ahat+1/(phi^2)*(1-alpha)*ahat*ahat;
      double phi = a.norm();

      Matrix3D res = new Matrix3D();
      res.setIdentity();

      if (phi != 0)
      {
         Matrix3D aHat = hat(a);
         double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));
         double alpha = Math.sin(phi) / phi;

         aHat.scale(0.5 * beta);
         res.add(aHat);
         aHat = hat(a);
         aHat.multiply(aHat);
         aHat.scale((1 - alpha) / MathTools.square(phi));
         res.add(aHat);
      }

      return res;
   }

   public static Matrix3D naive_ddexpSO3(Vector3DReadOnly x, Vector3DReadOnly z)
   {
      // The MATLAB code:
      //      hatx = hat(x);
      //      hatz = hat(z);
      //      phi = norm(x);
      //
      //      beta = sin(phi/2)^2/((phi/2)^2);
      //      alpha = sin(phi)/phi;
      //
      //      res = 0.5*beta*hatz...
      //      + 1/phi^2*(1-alpha)*(hatx*hatz+hatz*hatx)...
      //      + 1/phi^2*(alpha-beta)*(x'*z)*hatx...
      //                               + 1/phi^2*(beta/2-3/phi^2*(1-alpha))*(x'*z)*hatx*hatx;
      double phi = x.norm();
      Matrix3D hatx = hat(x);
      Matrix3D hatz = hat(z);

      Matrix3D hatxhatz = new Matrix3D();
      hatxhatz.set(hatx);
      hatxhatz.multiply(hatz);
      Matrix3D hatzhatx = new Matrix3D();
      hatzhatx.set(hatz);
      hatzhatx.multiply(hatx);

      double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));
      double alpha = Math.sin(phi) / phi;
      double phi2 = MathTools.square(phi);

      Matrix3D res = new Matrix3D();
      res.set(hatz);
      res.scale(0.5 * beta);

      Matrix3D temp = new Matrix3D();
      // 1/phi^2*(1-alpha)*(hatx*hatz+hatz*hatx)
      temp.set(hatxhatz);
      temp.add(hatzhatx);
      temp.scale((1.0 - alpha) / phi2);
      res.add(temp);

      // 1/phi^2*(alpha-beta)*(x'*z)*hatx
      temp.set(hatx);
      temp.scale(x.dot(z) * (alpha - beta) / phi2);
      res.add(temp);

      // 1/phi^2*(beta/2-3/phi^2*(1-alpha))*(x'*z)*hatx*hatx
      temp.set(hatx);
      temp.multiply(hatx);
      temp.scale(x.dot(z) * (beta / 2.0 - 3.0 / phi2 * (1.0 - alpha)) / phi2);
      res.add(temp);
      return res;
   }

   private static Matrix3D hat(Vector3DReadOnly a)
   {
      Matrix3D res = new Matrix3D();
      res.setToTildeForm(a);
      return res;
   }
}
