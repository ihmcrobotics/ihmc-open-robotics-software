package us.ihmc.robotics.geometry;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Matrix3d;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class RotationalInertiaCalculatorTest
{
   private static final double DELTA = 1e-3;
   private static final int ITERATIONS = 1000;
   private double maxRandomValue;
   private Random random;

   @Before
   public void setUp()
   {
      maxRandomValue = 1e3;
      random = new Random();
   }

   private double randomPositiveDouble()
   {
      return random.nextDouble() * maxRandomValue;
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testBasicCylinder()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         double mass = randomPositiveDouble();
         double radius = randomPositiveDouble();
         double height = randomPositiveDouble();

         int pick = random.nextInt(Axis.values().length);
         Axis axis = Axis.values()[pick];

         double[] IxxIyyIzz = RotationalInertiaCalculator.getIxxIyyIzzOfSolidCylinder(mass, radius, height, axis);
         double inCircleAxis0, inCircleAxis1, mainAxis;

         switch (axis)
         {
            case X :
               inCircleAxis0 = IxxIyyIzz[1];
               inCircleAxis1 = IxxIyyIzz[2];
               mainAxis = IxxIyyIzz[0];

               break;

            case Y :
               inCircleAxis0 = IxxIyyIzz[0];
               inCircleAxis1 = IxxIyyIzz[2];
               mainAxis = IxxIyyIzz[1];

               break;

            case Z :
               inCircleAxis0 = IxxIyyIzz[0];
               inCircleAxis1 = IxxIyyIzz[1];
               mainAxis = IxxIyyIzz[2];

               break;

            default :
               throw new RuntimeException("A new axis has been added to the Axis enum, but is not covered in the testcase.");
         }

         assertEquals(mass * (3.0 * radius * radius + height * height) / 12.0, inCircleAxis0, DELTA);
         assertEquals(mass * (3.0 * radius * radius + height * height) / 12.0, inCircleAxis1, DELTA);
         assertEquals(0.5 * mass * radius * radius, mainAxis, DELTA);
         
         Matrix3d rotationMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, height, axis);
         assertEquals(IxxIyyIzz[0], rotationMatrix.m00, DELTA);
         assertEquals(IxxIyyIzz[1], rotationMatrix.m11, DELTA);
         assertEquals(IxxIyyIzz[2], rotationMatrix.m22, DELTA);
         assertEquals(0.0, rotationMatrix.m01, DELTA);
         assertEquals(0.0, rotationMatrix.m02, DELTA);
         assertEquals(0.0, rotationMatrix.m10, DELTA);
         assertEquals(0.0, rotationMatrix.m12, DELTA);
         assertEquals(0.0, rotationMatrix.m20, DELTA);
         assertEquals(0.0, rotationMatrix.m21, DELTA);
      }
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testBasicCylinderNegativeMass()
   {
      double mass = -1.0;
      double radius = 1.0;
      double height = 1.0;
      Axis axis = Axis.Z;
      RotationalInertiaCalculator.getIxxIyyIzzOfSolidCylinder(mass, radius, height, axis);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testBasicCylinderNegativeRadius()
   {
      double mass = 1.0;
      double radius = -1.0;
      double height = 1.0;
      Axis axis = Axis.Z;
      RotationalInertiaCalculator.getIxxIyyIzzOfSolidCylinder(mass, radius, height, axis);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testBasicCylinderNegativeHeight()
   {
      double mass = 1.0;
      double radius = 1.0;
      double height = -1.0;
      Axis axis = Axis.Z;
      RotationalInertiaCalculator.getIxxIyyIzzOfSolidCylinder(mass, radius, height, axis);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSolidEllipsoid()
   {
	   for( int i = 0 ; i < ITERATIONS; i++)
	   {
		   double mass = randomPositiveDouble();
		   double xRadius = randomPositiveDouble();
		   double yRadius = randomPositiveDouble();
		   double zRadius = randomPositiveDouble();

		   double Ixx = mass * (MathTools.square(yRadius) + MathTools.square(zRadius)) / 5.0;
		   double Iyy = mass * (MathTools.square(xRadius) + MathTools.square(zRadius)) / 5.0;
		   double Izz = mass * (MathTools.square(yRadius) + MathTools.square(xRadius)) / 5.0;
		   
		   Matrix3d inertiaTensor = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);

		   assertEquals(Ixx, inertiaTensor.m00, DELTA);
		   assertEquals(Iyy, inertiaTensor.m11, DELTA);
		   assertEquals(Izz, inertiaTensor.m22, DELTA);
	   }
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSolidEllipsoidNegativeMass()
   {
	   double mass = -1.0;
	   double xRadius = 1.0;
	   double yRadius = 1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSolidEllipsoidNegativeXRadius()
   {
	   double mass = 1.0;
	   double xRadius = -1.0;
	   double yRadius = 1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSolidEllipsoidNegativeYRadius()
   {
	   double mass = 1.0;
	   double xRadius = 1.0;
	   double yRadius = -1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testSolidEllipsoidNegativeZRadius()
   {
	   double mass = 1.0;
	   double xRadius = 1.0;
	   double yRadius = 1.0;
	   double zRadius = -1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testRadiiOfGyration()
   {
	   for(int i = 0; i< ITERATIONS; i++)
	   {
		   double mass = randomPositiveDouble();
		   double xRadius = randomPositiveDouble();
		   double yRadius = randomPositiveDouble();
		   double zRadius = randomPositiveDouble();
		   
		   Matrix3d rotationalInertia = RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(mass, xRadius, yRadius, zRadius);

		   assertEquals(mass * (MathTools.square(yRadius) + MathTools.square(zRadius)), rotationalInertia.m00, DELTA);
		   assertEquals(mass * (MathTools.square(zRadius) + MathTools.square(xRadius)), rotationalInertia.m11, DELTA);
		   assertEquals(mass * (MathTools.square(xRadius) + MathTools.square(yRadius)), rotationalInertia.m22, DELTA);
		   assertEquals(0.0, rotationalInertia.m01, DELTA);
		   assertEquals(0.0, rotationalInertia.m02, DELTA);
		   assertEquals(0.0, rotationalInertia.m10, DELTA);
		   assertEquals(0.0, rotationalInertia.m12, DELTA);
		   assertEquals(0.0, rotationalInertia.m20, DELTA);
		   assertEquals(0.0, rotationalInertia.m21, DELTA);
	   }
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testRadiiOfGyrationNegativeMass()
   {
	   double mass = -1.0;
	   double xRadius = 1.0;
	   double yRadius = 1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testRadiiOfGyrationNegativeXRadius()
   {
	   double mass = 1.0;
	   double xRadius = -1.0;
	   double yRadius = 1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testRadiiOfGyrationNegativeYRadius()
   {
	   double mass = 1.0;
	   double xRadius = 1.0;
	   double yRadius = -1.0;
	   double zRadius = 1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(mass, xRadius, yRadius, zRadius);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testRadiiOfGyrationNegativeZRadius()
   {
	   double mass = 1.0;
	   double xRadius = 1.0;
	   double yRadius = 1.0;
	   double zRadius = -1.0;
	   
	   RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(mass, xRadius, yRadius, zRadius);
   }
}
