package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

public class MPTCGainsCalculatorTest
{

   @Test
   public void testComputeSystemCoriolisAndCentrifugalMatrix()
   {
      Random random = new Random(4576786);
      int numberOfJoints = 20;
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      rootBody.updateFramesRecursively();
      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

      MPTCGainsCalculator mptcGainsCalculator = new MPTCGainsCalculator(rootBody);
      DMatrixRMaj systemCoriolisAndCentrifugalMatrix = mptcGainsCalculator.computeSystemCoriolisAndCentrifugalMatrix();
      DMatrixRMaj jointVelocities = new DMatrixRMaj(nDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
      DMatrixRMaj actualJointTaus = new DMatrixRMaj(nDoFs, 1);
      CommonOps_DDRM.mult(systemCoriolisAndCentrifugalMatrix, jointVelocities, actualJointTaus);

      GravityCoriolisExternalWrenchMatrixCalculator calculator = new GravityCoriolisExternalWrenchMatrixCalculator(rootBody);
      calculator.compute();
      DMatrixRMaj expectedJointTaus = calculator.getJointTauMatrix();

      MecanoTestTools.assertDMatrixEquals(expectedJointTaus, actualJointTaus, 1.0e-12);
   }

}
