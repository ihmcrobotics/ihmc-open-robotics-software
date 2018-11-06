package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

public class CompositeRigidBodyMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{
   private Random random;

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

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
      Random random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator);

      sixDoFJoint.setJointConfiguration(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJointTwist.setIncludingFrame(sixDoFJoint.getJointTwist());
      sixDoFJointTwist.getLinearPart().set(RandomGeometry.nextVector3D(random));
      sixDoFJointTwist.getAngularPart().set(RandomGeometry.nextVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBodyBasics floating = ScrewTools.addRigidBody("floating", sixDoFJoint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(),
            RandomGeometry.nextVector3D(random));

      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();

      SpatialInertia inertia = new SpatialInertia(floating.getInertia());
      inertia.changeFrame(ReferenceFrame.getWorldFrame());
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(sixDoFJoint.getDegreesOfFreedom(), sixDoFJoint.getDegreesOfFreedom());
      inertia.get(inertiaMatrix);

      JUnitTools.assertMatrixEquals(inertiaMatrix, massMatrix, 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFloatingTree()
   {
      random = new Random(1982L);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator);

      sixDoFJoint.setJointConfiguration(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJointTwist.setIncludingFrame(sixDoFJoint.getJointTwist());
      sixDoFJointTwist.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
      sixDoFJointTwist.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      RigidBodyBasics floating = ScrewTools.addRigidBody("floating", sixDoFJoint, EuclidCoreRandomTools.nextDiagonalMatrix3D(random), random.nextDouble(),
                                                   EuclidCoreRandomTools.nextVector3D(random));

      random = new Random(1986L);
      setUpRandomTree(floating);

      MassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      massMatrixCalculator.compute();
   }

   private void setUpRandomTree(RigidBodyBasics elevator)
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
