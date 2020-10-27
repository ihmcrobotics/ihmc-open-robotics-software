package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.function.IntUnaryOperator;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

public class ForceEstimatorContactPoint
{
   private final RigidBodyBasics contactingLink;
   private final Tuple3DReadOnly contactPointOffset;
   private final GeometricJacobian contactPointJacobian;
   private final ReferenceFrame contactPointFrame;
   private final boolean assumeZeroTorque;
   private final int numberOfDecisionVariables;

   /** Maps indices of this contact point's jacobian to the whole system jacobian. Column i of this jacobian maps to column indexMap[i] of the system jacobian */
   private final int[] indexMap;

   private final FramePoint3D tempPoint = new FramePoint3D();

   public ForceEstimatorContactPoint(JointBasics[] joints, RigidBodyBasics contactingLink, Tuple3DReadOnly contactPointOffset, GeometricJacobian contactPointJacobian, boolean assumeZeroTorque)
   {
      this.contactingLink = contactingLink;
      this.contactPointOffset = contactPointOffset;
      this.contactPointJacobian = contactPointJacobian;
      this.indexMap = new int[contactPointJacobian.getJacobianMatrix().getNumCols()];
      this.assumeZeroTorque = assumeZeroTorque;
      this.numberOfDecisionVariables = assumeZeroTorque ? 3 : 6;

      this.contactPointFrame = new ReferenceFrame(contactingLink.getName() + "ContactFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            tempPoint.setIncludingFrame(contactingLink.getParentJoint().getFrameAfterJoint(), contactPointOffset);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            transformToParent.setTranslationAndIdentityRotation(tempPoint);
         }
      };

      contactPointJacobian.changeFrame(contactPointFrame);

      JointBasics[] jointPath = contactPointJacobian.getJointsInOrder();
      ToIntFunction<JointBasics> jointIndexFunction = joint -> IntStream.range(0, joints.length)
                                                                        .filter(i -> joint == joints[i])
                                                                        .findFirst()
                                                                        .orElseThrow(() -> new RuntimeException("Could not find joint"));
      IntUnaryOperator indexOffset = i -> IntStream.range(0, i).map(j -> joints[j].getDegreesOfFreedom()).sum();

      for (int jointIndex = 0, mappedIndex = 0; jointIndex < jointPath.length; jointIndex++)
      {
         JointBasics joint = jointPath[jointIndex];
         int offset = indexOffset.applyAsInt(jointIndexFunction.applyAsInt(joint));

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
         {
            indexMap[mappedIndex++] = offset + i;
         }
      }
   }

   public boolean getAssumeZeroTorque()
   {
      return assumeZeroTorque;
   }

   public int getNumberOfDecisionVariables()
   {
      return numberOfDecisionVariables;
   }

   public GeometricJacobian getContactPointJacobian()
   {
      return contactPointJacobian;
   }

   public Tuple3DReadOnly getContactPointOffset()
   {
      return contactPointOffset;
   }

   public ReferenceFrame getContactPointFrame()
   {
      return contactPointFrame;
   }

   public RigidBodyBasics getContactingLink()
   {
      return contactingLink;
   }

   public int getSystemJacobianIndex(int contactPointJacobianIndex)
   {
      return indexMap[contactPointJacobianIndex];
   }
}
