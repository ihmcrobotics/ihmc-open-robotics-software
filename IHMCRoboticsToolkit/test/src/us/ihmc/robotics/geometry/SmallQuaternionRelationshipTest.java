package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SmallQuaternionRelationshipTest
{
   // Test for some relationships that are used in QuaternionOrientation estimation.

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSmallAngleQuaternionDifferences()
   { 
      // If deltaQ = q X q_hat_inverse
      // and phi is rotation vector corresponding to q
      // and phi_hat is rotation vector corresponding to q_hat
      // and delta_phi is the rotation vector corresponding to deltaQ.
      // Then check that delta_phi is approximately phi - phi_hat - 0.5 * phi cross phi_hat.
      
      Vector3D vectorOne = new Vector3D(0.2, 0.4, 0.6);
      Vector3D vectorTwo = new Vector3D(-0.3, 0.4, 0.6);
      
      vectorOne.normalize();
      vectorTwo.normalize();
      
      double angleOne = 0.3;
      double angleTwo = 0.35;
      
      AxisAngle axisAngleOne = new AxisAngle(vectorOne, angleOne);
      AxisAngle axisAngleTwo = new AxisAngle(vectorTwo, angleTwo);
      
      Quaternion quaternionOne = new Quaternion();
      quaternionOne.set(axisAngleOne);
      
      Quaternion quaternionTwo = new Quaternion();
      quaternionTwo.set(axisAngleTwo);
      
      Quaternion quaternionError = new Quaternion(quaternionOne);
      quaternionError.multiplyConjugateOther(quaternionTwo);
      
      System.out.println("quaternionError = " + quaternionError);
      
      Vector3D rotationVectorOne = new Vector3D(vectorOne);
      rotationVectorOne.scale(angleOne);
      
      Vector3D rotationVectorTwo = new Vector3D(vectorTwo);
      rotationVectorTwo.scale(angleTwo);
      
      
      Vector3D crossOneTwo = new Vector3D();
      crossOneTwo.cross(rotationVectorOne, rotationVectorTwo);
      crossOneTwo.scale(-0.5);
      
      Vector3D rotationVectorError = new Vector3D(rotationVectorOne);
      rotationVectorError.sub(rotationVectorTwo);
      rotationVectorError.add(crossOneTwo);
      
      Vector3D errorVector = new Vector3D(rotationVectorError);
      double errorAngle = errorVector.length();
      errorVector.normalize();
      
      AxisAngle axisAngleError = new AxisAngle(errorVector, errorAngle);

      Quaternion quaternionErrorCheck = new Quaternion();
      quaternionErrorCheck.set(axisAngleError);
      
      System.out.println("quaternionErrorCheck = " + quaternionErrorCheck);

      
      Quaternion quaternionShouldBeOne = new Quaternion(quaternionError);
      quaternionShouldBeOne.multiplyConjugateOther(quaternionErrorCheck);
      
      System.out.println("quaternionShouldBeOne = " + quaternionShouldBeOne);
      
      AxisAngle axisAngleShouldBeOne = new AxisAngle();
      axisAngleShouldBeOne.set(quaternionShouldBeOne);
      
      System.out.println("axisAngleShouldBeOne = " + axisAngleShouldBeOne.getAngle());

      assertTrue(axisAngleShouldBeOne.getAngle() < 0.005);
   }

}
