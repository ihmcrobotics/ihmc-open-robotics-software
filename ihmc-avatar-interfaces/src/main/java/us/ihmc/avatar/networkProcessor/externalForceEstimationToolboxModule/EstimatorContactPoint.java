package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;

import java.util.function.IntUnaryOperator;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

public class EstimatorContactPoint
{
   /** Contacting rigid body */
   private final RigidBodyBasics rigidBody;

   /** Contact point position in the link's parent joint's "frame after joint" */
   private final Vector3D contactPointOffset = new Vector3D();

   /** Corresponding frame at contactPointOffset, aligned with the link's parent joint's "frame after joint */
   private final ReferenceFrame contactPointFrame;

   /** Jacobian associated with this contact point, the jacobian's frame is contactPointFrame */
   private final GeometricJacobian contactPointJacobian;

   /** Whether the contact point is assumed to have zero moment or not. Advised only to be true for floating base */
   private final boolean assumeZeroTorque;

   /** Convenience field to denote whether this contact point has 3 or 6 associated force variables  */
   private final int numberOfDecisionVariables;

   /** Used to compute the jacobian if assumeZeroTorque is true */
   private final PointJacobian pointJacobian = new PointJacobian();

   /** Maps indices of this contact point's jacobian to the whole system jacobian. Column i of this jacobian maps to column indexMap[i] of the system jacobian */
   private final int[] indexMap;

   private final FramePoint3D tempPoint = new FramePoint3D();

   EstimatorContactPoint(JointBasics[] joints, RigidBodyBasics rigidBody, boolean assumeZeroTorque)
   {
      this.rigidBody = rigidBody;
      this.assumeZeroTorque = assumeZeroTorque;
      this.numberOfDecisionVariables = assumeZeroTorque ? 3 : 6;

      RigidBodyBasics baseLink = joints[0].getPredecessor();
      this.contactPointJacobian = new GeometricJacobian(baseLink, rigidBody, baseLink.getBodyFixedFrame());
      this.indexMap = new int[contactPointJacobian.getJacobianMatrix().getNumCols()];

      this.contactPointFrame = new ReferenceFrame(rigidBody.getName() + "ContactFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            tempPoint.setIncludingFrame(rigidBody.getParentJoint().getFrameAfterJoint(), contactPointOffset);
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

   public DMatrixRMaj computeContactJacobian()
   {
      if(assumeZeroTorque)
      {
         ReferenceFrame baseFrame = contactPointJacobian.getBaseFrame();
         contactPointJacobian.changeFrame(baseFrame);
         contactPointJacobian.compute();

         tempPoint.setIncludingFrame(rigidBody.getParentJoint().getFrameAfterJoint(), contactPointOffset);
         pointJacobian.set(contactPointJacobian, tempPoint);
         pointJacobian.compute();
         return pointJacobian.getJacobianMatrix();
      }
      else
      {
         contactPointFrame.update();
         contactPointJacobian.compute();
         return contactPointJacobian.getJacobianMatrix();
      }
   }

   public void setContactPointOffset(FramePoint3D contactPointPosition)
   {
      contactPointPosition.changeFrame(rigidBody.getParentJoint().getFrameAfterJoint());
      this.contactPointOffset.set(contactPointPosition);
   }

   public void setContactPointOffset(Tuple3DReadOnly contactPointPosition)
   {
      this.contactPointOffset.set(contactPointPosition);
   }

   public boolean getAssumeZeroTorque()
   {
      return assumeZeroTorque;
   }

   public int getNumberOfDecisionVariables()
   {
      return numberOfDecisionVariables;
   }

   public Tuple3DReadOnly getContactPointOffset()
   {
      return contactPointOffset;
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public int getSystemJacobianIndex(int contactPointJacobianIndex)
   {
      return indexMap[contactPointJacobianIndex];
   }
}
