package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class GeoregressionConversionToolsTest
{
   private final static double eps = 1e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testTransformConversionFromGeoregressionToVecmath()
   {
      Random rand = new Random(123384L);
      for (int i = 0; i < 100000; i++)
      {
         double x,y,z;
         x = rand.nextGaussian();
         y = rand.nextGaussian();
         z = rand.nextGaussian();
         Point3D vecmathPoint = new Point3D(x,y,z);
         Point3D_F64 georegressionPoint = new Point3D_F64(x,y,z);
         
         Se3_F64 georegressionTransform = new Se3_F64();
         RigidBodyTransform vecmathTransform = new RigidBodyTransform();
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.generateRandomRotationMatrix(rand);
         
         georegressionTransform.setTranslation(rand.nextGaussian(), rand.nextGaussian(), rand.nextGaussian());
         georegressionTransform.setRotation(new DenseMatrix64F(new double[][]
         {
            {rotationMatrix.getM00(), rotationMatrix.getM01(), rotationMatrix.getM02()}, {rotationMatrix.getM10(), rotationMatrix.getM11(), rotationMatrix.getM12()},
            {rotationMatrix.getM20(), rotationMatrix.getM21(), rotationMatrix.getM22()}
         }));
         GeoregressionConversionTools.setVecmathTransformFromGeoregressionTransform(vecmathTransform, georegressionTransform);
//         georegressionTransform.print();
//         System.out.println(vecmathTransform);
         
         vecmathTransform.transform(vecmathPoint);

         Point3D_F64 georegressionTransformResultPoint = new Point3D_F64();
         SePointOps_F64.transform(georegressionTransform, georegressionPoint, georegressionTransformResultPoint);
         
         assertEquals(georegressionTransformResultPoint.x, vecmathPoint.getX(), eps);
         assertEquals(georegressionTransformResultPoint.y, vecmathPoint.getY(), eps);
         assertEquals(georegressionTransformResultPoint.z, vecmathPoint.getZ(), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testTransformConversionFromVecmathToGeoregression()
   {
      Random random = new Random(123384L);
      for (int i = 0; i < 100000; i++)
      {
         double x,y,z;
         x = random.nextGaussian();
         y = random.nextGaussian();
         z = random.nextGaussian();
         Point3D vecmathPoint = new Point3D(x,y,z);
         Point3D_F64 georegressionPoint = new Point3D_F64(x,y,z);
         
         Se3_F64 georegressionTransform = new Se3_F64();
         RigidBodyTransform vecmathTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         
         
         GeoregressionConversionTools.setGeoregressionTransformFromVecmath(vecmathTransform, georegressionTransform);
//         georegressionTransform.print();
//         System.out.println(vecmathTransform);
         
         vecmathTransform.transform(vecmathPoint);

         Point3D_F64 georegressionTransformResultPoint = new Point3D_F64();
         SePointOps_F64.transform(georegressionTransform, georegressionPoint, georegressionTransformResultPoint);
         
         assertEquals(georegressionTransformResultPoint.x, vecmathPoint.getX(), eps);
         assertEquals(georegressionTransformResultPoint.y, vecmathPoint.getY(), eps);
         assertEquals(georegressionTransformResultPoint.z, vecmathPoint.getZ(), eps);
      }
   }

}
