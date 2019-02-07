package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.random.RandomGeometry;

public class TotalMassCalculatorTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testComputeSubTreeMass()
   {
      Random random = new Random(100L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("body", worldFrame);
      int numberOfJoints = 100;
      double addedMass = createRandomRigidBodyTreeAndReturnTotalMass(worldFrame, elevator, numberOfJoints, random);


      assertEquals(addedMass, TotalMassCalculator.computeSubTreeMass(elevator), 0.00001);

   }

   public static double createRandomRigidBodyTreeAndReturnTotalMass(ReferenceFrame worldFrame, RigidBodyBasics elevator, int numberOfJoints, Random random)
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



         RigidBodyBasics inverseDynamicsParentBody;
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

         RevoluteJoint currentJoint = new RevoluteJoint("jointID" + i, inverseDynamicsParentBody, jointOffset, jointAxis);
         double jointPosition = random.nextDouble();
         currentJoint.setQ(jointPosition);
         currentJoint.setQd(0.0);
         currentJoint.setQdd(0.0);
         new RigidBody("bodyID" + i, currentJoint, momentOfInertia, mass, comOffset);

         potentialInverseDynamicsParentJoints.add(currentJoint);
      }

      elevator.updateFramesRecursively();

      return totalMass;
   }

}
