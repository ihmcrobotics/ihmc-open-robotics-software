package us.ihmc.jMonkeyEngineToolkit.jme.util;

import static junit.framework.Assert.*;

import java.util.Random;

import org.junit.Test;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.random.RandomGeometry;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/18/13
 * Time: 5:24 PM
 * To change this template use File | Settings | File Templates.
 */
public class JMEGeometryUtilsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetInverse()
   {
      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         RigidBodyTransform transform3D = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         
         Transform transform = JMEGeometryUtils.transformFromZupToJMECoordinates(transform3D);
         Transform transformInverse = JMEGeometryUtils.getInverse(transform);
         
         transform = transform.combineWithParent(transformInverse);

         assertTrue(JMEGeometryUtils.epsilonEquals(Transform.IDENTITY, transform , 1e-6));
         
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromJMECoordinatesToZup()
   {
      Transform transform3D = new Transform();
      transform3D.set( Transform.IDENTITY );
      transform3D.setTranslation( new Vector3f(1.0f, 0.0f, 0.0f));

      RigidBodyTransform transform = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform3D);

      // Unit vector in x
      Vector3D originalVector = new Vector3D(1.0, 0.0, 0.0);
      Vector3D originalVectorTransformedToZup = new Vector3D();
      transform.transform(originalVector, originalVectorTransformedToZup);

      Vector3D expectedAnswer = new Vector3D(0.0, 1.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Y
      originalVector = new Vector3D(0.0, 1.0, 0.0);
      originalVectorTransformedToZup = new Vector3D();
      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3D(0.0, 0.0, 1.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Z
      originalVector = new Vector3D(0.0, 0.0, 1.0);
      originalVectorTransformedToZup = new Vector3D();
      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3D(1.0, 0.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromJMECoordinatesToZupWithItsInverse()
   {
      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         RigidBodyTransform transform3Doriginal = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         Transform transform3d = JMEGeometryUtils.transformFromZupToJMECoordinates(transform3Doriginal );
         RigidBodyTransform transform3backToOriginal = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform3d);

         assertTrue(transform3backToOriginal.epsilonEquals(transform3Doriginal, 1e-6));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromJMECoordinatesToZupWith90RotAboutX()
   {
      Transform transform;
      RigidBodyTransform transform3D;
      Vector3D originalVector;

      //*****
      transform = new Transform();
      transform.set(Transform.IDENTITY);

      // 90 degree about JME x

      Quaternion quat = new Quaternion();
      quat.fromAngleAxis( (float)(Math.PI/2.0), new Vector3f(1.0f, 0.0f, 0.0f) );
      transform.setRotation( quat );

      // expected that this is
      transform3D = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform);

      AxisAngle axisAngleTransformed = new AxisAngle();

      QuaternionBasics quat4d = new us.ihmc.euclid.tuple4D.Quaternion();
      transform3D.getRotation(quat4d);
      axisAngleTransformed.set(quat4d);

      // Unit vector in x
      originalVector = new Vector3D(1.0, 0.0, 0.0);
      Vector3D originalVectorTransformedToZup = new Vector3D();

      transform3D.transform(originalVector, originalVectorTransformedToZup);

      Vector3D expectedAnswer = new Vector3D(0.0, 1.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Y
      originalVector = new Vector3D(0.0, 1.0, 0.0);
      originalVectorTransformedToZup = new Vector3D();

      transform3D.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3D(1.0, 0.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Z
      originalVector = new Vector3D(0.0, 0.0, 1.0);
      originalVectorTransformedToZup = new Vector3D();

      transform3D.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3D(0.0, 0.0, -1.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromZupToJMECoordinates()
   {
      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In world
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromJMECoordinatesToZupPoint()
   {
      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In JME
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In WORLD
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformFromZupToJMECoordinatesQuaternion()
   {
      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         AxisAngle axisAngle4d = RandomGeometry.nextAxisAngle(random);
         QuaternionBasics quat4d = new us.ihmc.euclid.tuple4D.Quaternion();
         quat4d.set(axisAngle4d);


      }

//      transformFromZupToJMECoordinates(Quaternion rotation)
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRotationsFromAndToJMEToZupCoordinates()
   {
      //JME to World
      Quaternion quaternion = JMEGeometryUtils.getRotationFromJMEToZupCoordinates();
      QuaternionBasics quat4d = JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(quaternion);

      RotationMatrix matrix3d = new RotationMatrix();
      matrix3d.set(quat4d);

      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In world
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

   }

   private static boolean areVectorsEqual(Vector3f vector3fa, Vector3f vector3fb)
   {
      if (Math.abs(vector3fa.getX() - vector3fb.getX()) > 1e-6)
         return false;
      if (Math.abs(vector3fa.getY() - vector3fb.getY()) > 1e-6)
         return false;
      if (Math.abs(vector3fa.getZ() - vector3fb.getZ()) > 1e-6)
         return false;

      return true;
   }

}
