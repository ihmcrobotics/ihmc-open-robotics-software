package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

public class CoriolisCalculatorTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testComputeSystemCoriolisAndCentrifugalMatrix()
   {
      Random random = new Random(4576786);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
         
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         rootBody.updateFramesRecursively();
         
         InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(rootBody);
         calculator.setConsiderJointAccelerations(false);
         calculator.compute();
         DMatrixRMaj expectedJointTaus = calculator.getJointTauMatrix();
         CoriolisCalculator coriolisCalculator = new CoriolisCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), true);
         coriolisCalculator.compute();
         DMatrixRMaj systemCoriolisAndCentrifugalMatrix = coriolisCalculator.getCoriolis();
         DMatrixRMaj jointVelocities = new DMatrixRMaj(nDoFs, 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
         DMatrixRMaj actualJointTaus = new DMatrixRMaj(nDoFs, 1);
         CommonOps_DDRM.mult(systemCoriolisAndCentrifugalMatrix, jointVelocities, actualJointTaus);
         
         
         MecanoTestTools.assertDMatrixEquals(expectedJointTaus, actualJointTaus, 1.0e-12);
      }
   }

}
