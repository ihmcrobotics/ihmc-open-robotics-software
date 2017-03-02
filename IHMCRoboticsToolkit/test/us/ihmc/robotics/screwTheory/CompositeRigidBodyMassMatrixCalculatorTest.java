package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.RigidBodyTransformGeneratorTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.random.RandomGeometryTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.testing.JUnitTools;

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

      sixDoFJoint.setPositionAndRotation(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(RandomGeometry.nextVector3D(random));
      sixDoFJointTwist.setAngularPart(RandomGeometry.nextVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBody floating = ScrewTools.addRigidBody("floating", sixDoFJoint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(),
            RandomGeometry.nextVector3D(random));

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

      sixDoFJoint.setPositionAndRotation(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(EuclidCoreRandomTools.generateRandomVector3D(random));
      sixDoFJointTwist.setAngularPart(EuclidCoreRandomTools.generateRandomVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBody floating = ScrewTools.addRigidBody("floating", sixDoFJoint, EuclidCoreRandomTools.generateRandomDiagonalMatrix3D(random), random.nextDouble(),
                                                   EuclidCoreRandomTools.generateRandomVector3D(random));

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

      Vector3D[] jointAxes1 = {X, Y, Z, Y, X};
      ScrewTestTools.createRandomChainRobot("chainA", joints, elevator, jointAxes1, random);

      Vector3D[] jointAxes2 = {Z, X, Y, X, X};
      ScrewTestTools.createRandomChainRobot("chainB", joints, elevator, jointAxes2, random);

      Vector3D[] jointAxes3 = {Y, Y, X, X, X};
      ScrewTestTools.createRandomChainRobot("chainC", joints, elevator, jointAxes3, random);
   }
}
