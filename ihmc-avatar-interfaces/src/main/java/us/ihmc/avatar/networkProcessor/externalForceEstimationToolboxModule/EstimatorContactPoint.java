package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;

import java.util.function.IntUnaryOperator;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

public class EstimatorContactPoint
{
   private final RigidBodyBasics rigidBody;
   private final FramePoint3D contactPointPosition = new FramePoint3D();
   private final FrameVector3D surfaceNormal = new FrameVector3D();
   private final FramePose3D surfacePose = new FramePose3D();
   private final PoseReferenceFrame contactPointFrame;

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

   public EstimatorContactPoint(JointBasics[] joints, RigidBodyBasics rigidBody, boolean assumeZeroTorque)
   {
      this.rigidBody = rigidBody;
      this.assumeZeroTorque = assumeZeroTorque;
      this.numberOfDecisionVariables = assumeZeroTorque ? 3 : 6;

      RigidBodyBasics baseLink = joints[0].getPredecessor();
      this.contactPointJacobian = new GeometricJacobian(baseLink, rigidBody, baseLink.getBodyFixedFrame());
      this.indexMap = new int[contactPointJacobian.getJacobianMatrix().getNumCols()];

      this.contactPointFrame = new PoseReferenceFrame(rigidBody.getName() + "ContactFrame", ReferenceFrame.getWorldFrame());

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

         contactPointPosition.changeFrame(contactPointFrame);
         pointJacobian.set(contactPointJacobian, contactPointPosition);
         pointJacobian.compute();
         return pointJacobian.getJacobianMatrix();
      }
      else
      {
         contactPointJacobian.compute();
         return contactPointJacobian.getJacobianMatrix();
      }
   }

   public void update()
   {
      contactPointPosition.changeFrame(ReferenceFrame.getWorldFrame());
      surfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

      surfacePose.getPosition().set(contactPointPosition);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, surfacePose.getOrientation());
      contactPointFrame.setPoseAndUpdate(surfacePose);
   }

   public boolean getAssumeZeroTorque()
   {
      return assumeZeroTorque;
   }

   public int getNumberOfDecisionVariables()
   {
      return numberOfDecisionVariables;
   }

   public FramePoint3D getContactPointPosition()
   {
      return contactPointPosition;
   }

   public FrameVector3D getSurfaceNormal()
   {
      return surfaceNormal;
   }

   public ReferenceFrame getContactPointFrame()
   {
      return contactPointFrame;
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
