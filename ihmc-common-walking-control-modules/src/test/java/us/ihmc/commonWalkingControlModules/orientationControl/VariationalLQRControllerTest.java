package us.ihmc.commonWalkingControlModules.orientationControl;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class VariationalLQRControllerTest
{
   @Test
   public void testWithNoFeedback()
   {
      VariationalLQRController controller = new VariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         QuaternionReadOnly desiredOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3DReadOnly desiredAngularVelocity = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));
         Vector3DReadOnly desiredTorque = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));

         controller.setDesired(desiredOrientation, desiredAngularVelocity, desiredTorque);
         controller.compute(desiredOrientation, desiredAngularVelocity);

         Vector3DBasics feedbackTorque = new Vector3D();
         controller.getDesiredTorque(feedbackTorque);

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(desiredTorque, feedbackTorque, 1e-5);
      }
   }

   @Test
   public void testWithEasyOrientationError()
   {
      VariationalLQRController controller = new VariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         QuaternionReadOnly desiredOrientation = new Quaternion();
         Vector3DReadOnly desiredAngularVelocity = new Vector3D();
//         Vector3DReadOnly desiredTorque = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));
         Vector3D desiredTorque = new Vector3D();

         QuaternionBasics pitchedOrientation = new Quaternion(desiredOrientation);
         QuaternionBasics yawedOrientation = new Quaternion(desiredOrientation);
         QuaternionBasics rolledOrientation = new Quaternion(desiredOrientation);

         double pitchRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double yawRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double rollRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));

         pitchedOrientation.setToPitchOrientation(pitchRotation);
         yawedOrientation.setToYawOrientation(yawRotation);
         rolledOrientation.setToRollOrientation(rollRotation);

         controller.setDesired(desiredOrientation, desiredAngularVelocity, desiredTorque);
         Vector3DBasics feedbackTorque = new Vector3D();

         // check yaw only
         controller.compute(yawedOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(-Math.signum(yawRotation), Math.signum(feedbackTorque.getZ()), 1e-5);

         // check pitch only
         controller.compute(pitchedOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(-Math.signum(pitchRotation), Math.signum(feedbackTorque.getY()), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);


         // check roll only
         controller.compute(rolledOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(-Math.signum(rollRotation), Math.signum(feedbackTorque.getX()), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);
      }
   }

   @Test
   public void testWithEasyAngularVelocityError()
   {
      VariationalLQRController controller = new VariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         QuaternionReadOnly desiredOrientation = new Quaternion();
         Vector3DReadOnly desiredAngularVelocity = new Vector3D();
         Vector3D desiredTorque = new Vector3D();

         Vector3D pitchedVelocity = new Vector3D();
         Vector3D yawedVelocity = new Vector3D();
         Vector3D rolledVelocity = new Vector3D();

         double pitchVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double yawVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double rollVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));

         pitchedVelocity.setY(pitchVelocity);
         rolledVelocity.setX(rollVelocity);
         yawedVelocity.setZ(yawVelocity);

         controller.setDesired(desiredOrientation, desiredAngularVelocity, desiredTorque);
         Vector3DBasics feedbackTorque = new Vector3D();

         // check yaw only
         controller.compute(desiredOrientation, yawedVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(-Math.signum(yawVelocity), Math.signum(feedbackTorque.getZ()), 1e-5);

         // check pitch only
         controller.compute(desiredOrientation, pitchedVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(-Math.signum(pitchVelocity), Math.signum(feedbackTorque.getY()), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);

         // check roll only
         controller.compute(desiredOrientation, rolledVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(-Math.signum(rollVelocity), Math.signum(feedbackTorque.getX()), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);
      }
   }
}
