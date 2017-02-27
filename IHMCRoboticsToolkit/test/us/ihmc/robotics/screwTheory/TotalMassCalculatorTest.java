package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TotalMassCalculatorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubTreeMass()
   {
      Random random = new Random(100L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("body", elevatorFrame);
      int numberOfJoints = 100;
      double addedMass = createRandomRigidBodyTreeAndReturnTotalMass(worldFrame, elevator, numberOfJoints, random);


      assertEquals(addedMass, TotalMassCalculator.computeSubTreeMass(elevator), 0.00001);

   }

   public static double createRandomRigidBodyTreeAndReturnTotalMass(ReferenceFrame worldFrame, RigidBody elevator, int numberOfJoints, Random random)
   {
      double totalMass = 0.0;
      boolean rootAdded = false;
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>();    // synchronized with potentialParentJoints

      for (int i = 0; i < numberOfJoints; i++)
      {
         Vector3D jointOffset = RandomGeometry.nextVector3D(random);
         Vector3D jointAxis = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         jointAxis.normalize();
         Matrix3D momentOfInertia = RandomGeometry.nextDiagonalMatrix3D(random);
         double mass = random.nextDouble();
         totalMass += mass;
         Vector3D comOffset = RandomGeometry.nextVector3D(random);



         RigidBody inverseDynamicsParentBody;
         if (!rootAdded)
         {
            rootAdded = true;
            inverseDynamicsParentBody = elevator;
         }
         else
         {
            int parentIndex = random.nextInt(potentialInverseDynamicsParentJoints.size());
            RevoluteJoint inverseDynamicsParentJoint = potentialInverseDynamicsParentJoints.get(parentIndex);
            inverseDynamicsParentBody = inverseDynamicsParentJoint.getSuccessor();
         }

         RevoluteJoint currentJoint = ScrewTools.addRevoluteJoint("jointID" + i, inverseDynamicsParentBody, jointOffset, jointAxis);
         double jointPosition = random.nextDouble();
         currentJoint.setQ(jointPosition);
         currentJoint.setQd(0.0);
         currentJoint.setQddDesired(0.0);
         ScrewTools.addRigidBody("bodyID" + i, currentJoint, momentOfInertia, mass, comOffset);

         potentialInverseDynamicsParentJoints.add(currentJoint);
      }

      elevator.updateFramesRecursively();

      return totalMass;
   }

}
