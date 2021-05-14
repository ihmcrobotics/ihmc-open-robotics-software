package us.ihmc.commonWalkingControlModules.orientationControl;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

import java.util.Random;

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
}
