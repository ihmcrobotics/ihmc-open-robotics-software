package us.ihmc.robotics.geometry;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

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
         RigidBodyTransform transform1 = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform transform2 = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         transform1.setTranslation(new Vector3D());
         transform2.setTranslation(new Vector3D());

         verifyRelationship(transform1, transform2);
      }
      
   }

   private void verifyRelationship(RigidBodyTransform transform1, RigidBodyTransform transform2)
   {
      Quaternion quaternion1 = new Quaternion();
      Quaternion quaternion2 = new Quaternion();

      transform1.getRotation(quaternion1);
      transform2.getRotation(quaternion2);
      
      // Going through the quaternion multiplication method:
      Quaternion quaternion1TimesQuaternion2TimesQuaternion1Inverse = computeQuat1Quat2Quat1Conjugate(quaternion1, quaternion2);
      
      // Now going through the rotation vector (axis-angle as a single vector) method:
      Quaternion rotatedAxisAngle2Quaternion = computeRotatedRotationVector(transform1, quaternion1, quaternion2);
      
      EuclidCoreTestTools.assertQuaternionEquals(quaternion1TimesQuaternion2TimesQuaternion1Inverse, rotatedAxisAngle2Quaternion, 1e-7);
//      System.out.println("quaternion1TimesQuaternion2TimesQuaternion1Inverse = " + quaternion1TimesQuaternion2TimesQuaternion1Inverse);
//      System.out.println("rotatedAxisAngle2Quaternion = " + rotatedAxisAngle2Quaternion);
   }

   private Quaternion computeRotatedRotationVector(RigidBodyTransform transform1, Quaternion quaternion1, Quaternion quaternion2)
   {
      AxisAngle axisAngle1 = new AxisAngle();
      axisAngle1.set(quaternion1);
      
      AxisAngle axisAngle2 = new AxisAngle();
      axisAngle2.set(quaternion2);
      
      Vector3D axisAngleVector2 = axisAngleToVector(axisAngle2);
      
      Vector3D rotatedAxisAngleVector2 = new Vector3D(axisAngleVector2);
      transform1.transform(rotatedAxisAngleVector2);
      
      Vector3D rotatedAxisAngleVector2Normalized = new Vector3D(rotatedAxisAngleVector2);
      rotatedAxisAngleVector2Normalized.normalize();
      
      double rotatedAxisAngleVector2Magnitude = rotatedAxisAngleVector2.length();
      
      AxisAngle rotatedAxisAngle2 = new AxisAngle(rotatedAxisAngleVector2Normalized, rotatedAxisAngleVector2Magnitude);
      
      Quaternion rotatedAxisAngle2Quaternion = new Quaternion();
      rotatedAxisAngle2Quaternion.set(rotatedAxisAngle2);
      return rotatedAxisAngle2Quaternion;
   }

   private Quaternion computeQuat1Quat2Quat1Conjugate(Quaternion quaternion1, Quaternion quaternion2)
   {
      Quaternion quaternion1Inverse = new Quaternion(quaternion1);
      quaternion1Inverse.inverse();
      
      Quaternion quaternion1TimesQuaternion2 = new Quaternion();
      quaternion1TimesQuaternion2.multiply(quaternion1, quaternion2); 
      
      Quaternion quaternion1TimesQuaternion2TimesQuaternion1Inverse = new Quaternion();
      quaternion1TimesQuaternion2TimesQuaternion1Inverse.multiply(quaternion1TimesQuaternion2, quaternion1Inverse);
      return quaternion1TimesQuaternion2TimesQuaternion1Inverse;
   }
   
   private static Vector3D axisAngleToVector(AxisAngle axisAngle)
   {
      Vector3D ret = new Vector3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ());
      ret.scale(axisAngle.getAngle());
      
      return ret;
   }
   
}
