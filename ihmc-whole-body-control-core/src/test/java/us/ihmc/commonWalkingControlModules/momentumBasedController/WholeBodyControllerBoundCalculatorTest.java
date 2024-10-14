package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DMatrixRMaj;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class WholeBodyControllerBoundCalculatorTest
{
   @Test
   public void testComputeDefaultBoundsForOneJoint()
   {
      double dt = 1e-2;
      RigidBodyBasics root = new RigidBody("root", ReferenceFrame.getWorldFrame());
      OneDoFJoint joint = new RevoluteJoint("joint", root, new Vector3D());
      JointIndexHandler indexHandler = new JointIndexHandler(new OneDoFJoint[]{joint});

      double qMax = 1.0;
      double qMin = -1.0;

      joint.setJointLimitLower(qMin);
      joint.setJointLimitUpper(qMax);

      WholeBodyControllerBoundCalculator calculator = new WholeBodyControllerBoundCalculator(indexHandler, dt, true, new YoRegistry("test"));


      joint.setQ(0.0);
      joint.setQd(0.0);

      double qddotAbsolute = 200.0;

      DMatrixRMaj qddotMin = new DMatrixRMaj(1, 1);
      DMatrixRMaj qddotMax = new DMatrixRMaj(1, 1);

      calculator.computeJointAccelerationLimits(qddotAbsolute, qddotMin, qddotMax);

      assertEquals(-qddotAbsolute, qddotMin.get(0, 0), 1e-6);
      assertEquals(qddotAbsolute, qddotMax.get(0, 0), 1e-6);

      joint.setQ(-1.5);

      calculator.computeJointAccelerationLimits(qddotAbsolute, qddotMin, qddotMax);

      assertEquals(0.0, qddotMin.get(0, 0), 1e-6);
      assertEquals(qddotAbsolute, qddotMax.get(0, 0), 1e-6);

      joint.setQ(1.5);

      calculator.computeJointAccelerationLimits(qddotAbsolute, qddotMin, qddotMax);

      assertEquals(-qddotAbsolute, qddotMin.get(0, 0), 1e-6);
      assertEquals(0.0, qddotMax.get(0, 0), 1e-6);

      double qd = 5.0;
      double startQ = qMax - qd * dt - 0.5 * qddotAbsolute * dt * dt;
      for (double q = startQ; q < qMax; q += 0.001)
      {
         double qddMaxExpected = Math.max(2.0 * (qMax - q - qd * dt) / (dt * dt), 0.0); // simple integration of the current state
         joint.setQ(q);
         joint.setQd(qd);

         calculator.computeJointAccelerationLimits(qddotAbsolute, qddotMin, qddotMax);

         assertEquals(qddMaxExpected, qddotMax.get(0, 0), 1e-6, "error at q = " + q);
      }
   }
}
