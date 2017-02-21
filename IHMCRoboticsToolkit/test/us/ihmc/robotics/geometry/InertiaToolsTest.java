package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomTools;

public class InertiaToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double DELTA = 1e-3;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

         Matrix3D rotationalInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);
         Vector3D principalMomentsOfInertia = new Vector3D(rotationalInertia.getM00(), rotationalInertia.getM11(), rotationalInertia.getM22());

         Vector3D ellipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

         assertEquals(xRadius, ellipsoidRadii.getX(), DELTA);
         assertEquals(yRadius, ellipsoidRadii.getY(), DELTA);
         assertEquals(zRadius, ellipsoidRadii.getZ(), DELTA);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

         Matrix3D rotationalInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, xRadius, yRadius, zRadius);

         Matrix3D rotationalInertiaCopy = new Matrix3D(rotationalInertia);
         RotationMatrix inertialFrameRotation = RandomTools.generateRandomRotationMatrix(random);
         
         Matrix3D rotatedInertia = InertiaTools.rotate(inertialFrameRotation, rotationalInertiaCopy);
         
         RotationMatrix principalAxesAfterRotation = new RotationMatrix();
         Vector3D principalMomentsOfInertiaAfterRotation = new Vector3D();
         InertiaTools.computePrincipalMomentsOfInertia(rotatedInertia, principalAxesAfterRotation, principalMomentsOfInertiaAfterRotation);

         ArrayList<Double> principleMomentsBeforeRotation = new ArrayList<Double>();
         principleMomentsBeforeRotation.add(rotationalInertia.getM00());
         principleMomentsBeforeRotation.add(rotationalInertia.getM11());
         principleMomentsBeforeRotation.add(rotationalInertia.getM22());
         
         ArrayList<Double> principleMomentsAfterRotation = new ArrayList<Double>();
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getX());
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getY());
         principleMomentsAfterRotation.add(principalMomentsOfInertiaAfterRotation.getZ());
         
         Collections.sort(principleMomentsBeforeRotation);
         Collections.sort(principleMomentsAfterRotation);
         
         assertEquals(principleMomentsBeforeRotation.get(0), principleMomentsAfterRotation.get(0), epsilon);
         assertEquals(principleMomentsBeforeRotation.get(1), principleMomentsAfterRotation.get(1), epsilon);
         assertEquals(principleMomentsBeforeRotation.get(2), principleMomentsAfterRotation.get(2), epsilon);

         Matrix3D inertiaAboutPrincipalAxes = new Matrix3D();
         inertiaAboutPrincipalAxes.setM00(principalMomentsOfInertiaAfterRotation.getX());
         inertiaAboutPrincipalAxes.setM11(principalMomentsOfInertiaAfterRotation.getY());
         inertiaAboutPrincipalAxes.setM22(principalMomentsOfInertiaAfterRotation.getZ());
         
         Matrix3D rotatedInertiaAgain = InertiaTools.rotate(principalAxesAfterRotation, inertiaAboutPrincipalAxes);
         assertTrue(rotatedInertiaAgain.epsilonEquals(rotatedInertia, epsilon));
      }
   }

}
