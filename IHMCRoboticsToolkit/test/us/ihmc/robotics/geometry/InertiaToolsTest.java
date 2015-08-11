package us.ihmc.robotics.geometry;

import org.junit.Test;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class InertiaToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double DELTA = 1e-3;

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetInertiaEllipsoidRadii()
   {
      Random random = new Random();
      double maxRandomValue = 1000.0;
      for (int i = 0; i < ITERATIONS; i++)
      {
         double mass = maxRandomValue * random.nextDouble();
         double xRadius = maxRandomValue * random.nextDouble();
         double yRadius = maxRandomValue * random.nextDouble();
         double zRadius = maxRandomValue * random.nextDouble();

         Matrix3d rotationalInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
         Vector3d principalMomentsOfInertia = new Vector3d(rotationalInertia.m00, rotationalInertia.m11, rotationalInertia.m22);

         Vector3d ellipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

         assertEquals(xRadius, ellipsoidRadii.x, DELTA);
         assertEquals(yRadius, ellipsoidRadii.y, DELTA);
         assertEquals(zRadius, ellipsoidRadii.z, DELTA);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testRotations()
   {
      double epsilon = 1e-7;
      
      Random random = new Random();
      double maxRandomValue = 1.0;
      for (int i = 0; i < ITERATIONS; i++)
      {
         double mass = maxRandomValue * random.nextDouble();
         double xRadius = maxRandomValue * random.nextDouble();
         double yRadius = maxRandomValue * random.nextDouble();
         double zRadius = maxRandomValue * random.nextDouble();

         Matrix3d rotationalInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);

         Matrix3d rotationalInertiaCopy = new Matrix3d(rotationalInertia);
         Matrix3d inertialFrameRotation = RandomTools.generateRandomRotationMatrix3d(random);
         
         Matrix3d rotatedInertia = InertiaTools.rotate(inertialFrameRotation, rotationalInertiaCopy);
         
         Matrix3d principalAxesAfterRotation = new Matrix3d();
         Vector3d principalMomentsOfInertiaAfterRotation = new Vector3d();
         InertiaTools.computePrincipalMomentsOfInertia(rotatedInertia, principalAxesAfterRotation, principalMomentsOfInertiaAfterRotation);

         ArrayList<Double> principleMomentsBeforeRotation = new ArrayList<Double>();
         principleMomentsBeforeRotation.add(rotationalInertia.m00);
         principleMomentsBeforeRotation.add(rotationalInertia.m11);
         principleMomentsBeforeRotation.add(rotationalInertia.m22);
         
         ArrayList<Double> principleMomentsAfterRotation = new ArrayList<Double>();
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getX());
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getY());
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getZ());
         
         Collections.sort(principleMomentsBeforeRotation);
         Collections.sort(principleMomentsAfterRotation);
         
         assertEquals(principleMomentsBeforeRotation.get(0), principleMomentsAfterRotation.get(0), epsilon);
         assertEquals(principleMomentsBeforeRotation.get(1), principleMomentsAfterRotation.get(1), epsilon);
         assertEquals(principleMomentsBeforeRotation.get(2), principleMomentsAfterRotation.get(2), epsilon);

         Matrix3d inertiaAboutPrincipalAxes = new Matrix3d();
         inertiaAboutPrincipalAxes.m00 = principalMomentsOfInertiaAfterRotation.getX();
         inertiaAboutPrincipalAxes.m11 = principalMomentsOfInertiaAfterRotation.getY();
         inertiaAboutPrincipalAxes.m22 = principalMomentsOfInertiaAfterRotation.getZ();
         
         Matrix3d rotatedInertiaAgain = InertiaTools.rotate(principalAxesAfterRotation, inertiaAboutPrincipalAxes);
         assertTrue(rotatedInertiaAgain.epsilonEquals(rotatedInertia, epsilon));
      }
   }

}
