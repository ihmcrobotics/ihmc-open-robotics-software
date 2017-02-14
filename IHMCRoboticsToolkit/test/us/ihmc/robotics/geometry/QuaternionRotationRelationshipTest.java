package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

public class QuaternionRotationRelationshipTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testQuaternionRotationRelationship()
   {
      Random random = new Random(1776L);
      
      // Validating that q1 times q2 times q1_congugate = zeta(R(q1) zeta_inverse(q2))  
      // where R(q) is the rotation matrix corresponding to q, 
      // and zeta is the conversion from quaternion to a rotation vector (theta u_hat)

      int numberOfTests = 1000;

      for (int i=0; i<numberOfTests; i++)
      {
         RigidBodyTransform transform1 = RigidBodyTransform.generateRandomTransform(random);
         RigidBodyTransform transform2 = RigidBodyTransform.generateRandomTransform(random);
         transform1.setTranslation(new Vector3d());
         transform2.setTranslation(new Vector3d());

         verifyRelationship(transform1, transform2);
      }
      
   }

   private void verifyRelationship(RigidBodyTransform transform1, RigidBodyTransform transform2)
   {
      Quat4d quaternion1 = new Quat4d();
      Quat4d quaternion2 = new Quat4d();

      transform1.getRotation(quaternion1);
      transform2.getRotation(quaternion2);
      
      // Going through the quaternion multiplication method:
      Quat4d quaternion1TimesQuaternion2TimesQuaternion1Inverse = computeQuat1Quat2Quat1Conjugate(quaternion1, quaternion2);
      
      // Now going through the rotation vector (axis-angle as a single vector) method:
      Quat4d rotatedAxisAngle2Quaternion = computeRotatedRotationVector(transform1, quaternion1, quaternion2);
      
      JUnitTools.assertQuaternionsEqual(quaternion1TimesQuaternion2TimesQuaternion1Inverse, rotatedAxisAngle2Quaternion, 1e-7);
//      System.out.println("quaternion1TimesQuaternion2TimesQuaternion1Inverse = " + quaternion1TimesQuaternion2TimesQuaternion1Inverse);
//      System.out.println("rotatedAxisAngle2Quaternion = " + rotatedAxisAngle2Quaternion);
   }

   private Quat4d computeRotatedRotationVector(RigidBodyTransform transform1, Quat4d quaternion1, Quat4d quaternion2)
   {
      AxisAngle4d axisAngle1 = new AxisAngle4d();
      axisAngle1.set(quaternion1);
      
      AxisAngle4d axisAngle2 = new AxisAngle4d();
      axisAngle2.set(quaternion2);
      
      Vector3d axisAngleVector2 = axisAngleToVector(axisAngle2);
      
      Vector3d rotatedAxisAngleVector2 = new Vector3d(axisAngleVector2);
      transform1.transform(rotatedAxisAngleVector2);
      
      Vector3d rotatedAxisAngleVector2Normalized = new Vector3d(rotatedAxisAngleVector2);
      rotatedAxisAngleVector2Normalized.normalize();
      
      double rotatedAxisAngleVector2Magnitude = rotatedAxisAngleVector2.length();
      
      AxisAngle4d rotatedAxisAngle2 = new AxisAngle4d(rotatedAxisAngleVector2Normalized, rotatedAxisAngleVector2Magnitude);
      
      Quat4d rotatedAxisAngle2Quaternion = new Quat4d();
      rotatedAxisAngle2Quaternion.set(rotatedAxisAngle2);
      return rotatedAxisAngle2Quaternion;
   }

   private Quat4d computeQuat1Quat2Quat1Conjugate(Quat4d quaternion1, Quat4d quaternion2)
   {
      Quat4d quaternion1Inverse = new Quat4d(quaternion1);
      quaternion1Inverse.inverse();
      
      Quat4d quaternion1TimesQuaternion2 = new Quat4d();
      quaternion1TimesQuaternion2.mul(quaternion1, quaternion2); 
      
      Quat4d quaternion1TimesQuaternion2TimesQuaternion1Inverse = new Quat4d();
      quaternion1TimesQuaternion2TimesQuaternion1Inverse.mul(quaternion1TimesQuaternion2, quaternion1Inverse);
      return quaternion1TimesQuaternion2TimesQuaternion1Inverse;
   }
   
   private static Vector3d axisAngleToVector(AxisAngle4d axisAngle)
   {
      Vector3d ret = new Vector3d(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ());
      ret.scale(axisAngle.getAngle());
      
      return ret;
   }
   
}
