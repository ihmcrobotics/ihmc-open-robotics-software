package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class CompositeRigidBodyMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testKineticEnergy()
   {
      setUpRandomChainRobot();
      double expectedKineticEnergy = computeKineticEnergy(joints);

      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      double kineticEnergyFromMassMatrix = computeKineticEnergy(joints, massMatrix);

      assertEquals(expectedKineticEnergy, kineticEnergyFromMassMatrix, 1e-12);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSixDoFJoint()
   {
      Random random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevator.getBodyFixedFrame());

      sixDoFJoint.setPositionAndRotation(RigidBodyTransform.generateRandomTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.packJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(RandomTools.generateRandomVector(random));
      sixDoFJointTwist.setAngularPart(RandomTools.generateRandomVector(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBody floating = ScrewTools.addRigidBody("floating", sixDoFJoint, RandomTools.generateRandomDiagonalMatrix3d(random), random.nextDouble(),
            RandomTools.generateRandomVector(random));
      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();

      RigidBodyInertia inertia = floating.getInertiaCopy();
      inertia.changeFrame(ReferenceFrame.getWorldFrame());
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(sixDoFJoint.getDegreesOfFreedom(), sixDoFJoint.getDegreesOfFreedom());
      inertia.packMatrix(inertiaMatrix);

      JUnitTools.assertMatrixEquals(inertiaMatrix, massMatrix, 1e-5);
   }
}
