package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

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
      
      Vector3d vectorOne = new Vector3d(0.2, 0.4, 0.6);
      Vector3d vectorTwo = new Vector3d(-0.3, 0.4, 0.6);
      
      vectorOne.normalize();
      vectorTwo.normalize();
      
      double angleOne = 0.3;
      double angleTwo = 0.35;
      
      AxisAngle4d axisAngleOne = new AxisAngle4d(vectorOne, angleOne);
      AxisAngle4d axisAngleTwo = new AxisAngle4d(vectorTwo, angleTwo);
      
      Quat4d quaternionOne = new Quat4d();
      quaternionOne.set(axisAngleOne);
      
      Quat4d quaternionTwo = new Quat4d();
      quaternionTwo.set(axisAngleTwo);
      
      Quat4d quaternionError = new Quat4d();
      quaternionError.mulInverse(quaternionOne, quaternionTwo);
      
      System.out.println("quaternionError = " + quaternionError);
      
      Vector3d rotationVectorOne = new Vector3d(vectorOne);
      rotationVectorOne.scale(angleOne);
      
      Vector3d rotationVectorTwo = new Vector3d(vectorTwo);
      rotationVectorTwo.scale(angleTwo);
      
      
      Vector3d crossOneTwo = new Vector3d();
      crossOneTwo.cross(rotationVectorOne, rotationVectorTwo);
      crossOneTwo.scale(-0.5);
      
      Vector3d rotationVectorError = new Vector3d(rotationVectorOne);
      rotationVectorError.sub(rotationVectorTwo);
      rotationVectorError.add(crossOneTwo);
      
      Vector3d errorVector = new Vector3d(rotationVectorError);
      double errorAngle = errorVector.length();
      errorVector.normalize();
      
      AxisAngle4d axisAngleError = new AxisAngle4d(errorVector, errorAngle);

      Quat4d quaternionErrorCheck = new Quat4d();
      quaternionErrorCheck.set(axisAngleError);
      
      System.out.println("quaternionErrorCheck = " + quaternionErrorCheck);

      
      Quat4d quaternionShouldBeOne = new Quat4d();
      quaternionShouldBeOne.mulInverse(quaternionError, quaternionErrorCheck);
      
      System.out.println("quaternionShouldBeOne = " + quaternionShouldBeOne);
      
      AxisAngle4d axisAngleShouldBeOne = new AxisAngle4d();
      axisAngleShouldBeOne.set(quaternionShouldBeOne);
      
      System.out.println("axisAngleShouldBeOne = " + axisAngleShouldBeOne.getAngle());

      assertTrue(axisAngleShouldBeOne.getAngle() < 0.005);
   }

}
