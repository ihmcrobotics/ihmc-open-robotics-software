package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class CompositeRigidBodyMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{
   private Random random;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSixDoFJoint()
   {
      random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevator.getBodyFixedFrame());

      sixDoFJoint.setPositionAndRotation(RigidBodyTransform.generateRandomTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
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
      inertia.getMatrix(inertiaMatrix);

      JUnitTools.assertMatrixEquals(inertiaMatrix, massMatrix, 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFloatingTree()
   {
      random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevator.getBodyFixedFrame());

      sixDoFJoint.setPositionAndRotation(RigidBodyTransform.generateRandomTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(RandomTools.generateRandomVector(random));
      sixDoFJointTwist.setAngularPart(RandomTools.generateRandomVector(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBody floating = ScrewTools.addRigidBody("floating", sixDoFJoint, RandomTools.generateRandomDiagonalMatrix3d(random), random.nextDouble(),
                                                   RandomTools.generateRandomVector(random));

      random = new Random(1986L);
      setUpRandomTree(floating);

      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();

      RigidBodyInertia inertia = floating.getInertiaCopy();
      inertia.changeFrame(ReferenceFrame.getWorldFrame());
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(sixDoFJoint.getDegreesOfFreedom(), sixDoFJoint.getDegreesOfFreedom());
      inertia.getMatrix(inertiaMatrix);
   }

   private void setUpRandomTree(RigidBody elevator)
   {
      joints = new ArrayList<RevoluteJoint>();

      Vector3d[] jointAxes1 = {X, Y, Z, Y, X};
      ScrewTestTools.createRandomChainRobot("chainA", joints, elevator, jointAxes1, random);

      Vector3d[] jointAxes2 = {Z, X, Y, X, X};
      ScrewTestTools.createRandomChainRobot("chainB", joints, elevator, jointAxes2, random);

      Vector3d[] jointAxes3 = {Y, Y, X, X, X};
      ScrewTestTools.createRandomChainRobot("chainC", joints, elevator, jointAxes3, random);
   }
}
