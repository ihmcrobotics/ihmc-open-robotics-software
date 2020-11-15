package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;

/**
 * Contact point object used by {@link ContactParticleFilter}
 */
public class ContactPointParticle
{
   private RigidBodyBasics rigidBody;
   private final RigidBodyBasics baseLink;
   private final JointBasics[] orderedJoints;

   private final FramePoint3D contactPointPosition = new FramePoint3D();
   private final FrameVector3D surfaceNormal = new FrameVector3D();
   private final FramePose3D surfacePose = new FramePose3D();
   private final PoseReferenceFrame contactPointFrame;

   private GeometricJacobian contactPointJacobian;
   private final PointJacobian pointJacobian = new PointJacobian();
   private int[] indexMap;

   public ContactPointParticle(String namePrefix, JointBasics[] orderedJoints)
   {
      this.orderedJoints = orderedJoints;

      baseLink = orderedJoints[0].getPredecessor();
      contactPointFrame = new PoseReferenceFrame(namePrefix + "ContactFrame", ReferenceFrame.getWorldFrame());
   }

   public void setRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;

      contactPointJacobian = new GeometricJacobian(baseLink, rigidBody, baseLink.getBodyFixedFrame());
      contactPointJacobian.changeFrame(contactPointFrame);
      indexMap = ExternalForceEstimationTools.createIndexMap(contactPointJacobian, orderedJoints);
   }

   public DMatrixRMaj computeContactJacobian()
   {
      assertInitialized();

      ReferenceFrame baseFrame = contactPointJacobian.getBaseFrame();
      contactPointJacobian.changeFrame(baseFrame);
      contactPointJacobian.compute();

      contactPointPosition.changeFrame(contactPointFrame);
      pointJacobian.set(contactPointJacobian, contactPointPosition);
      pointJacobian.compute();
      return pointJacobian.getJacobianMatrix();
   }

   public void update()
   {
      assertInitialized();

      contactPointPosition.changeFrame(ReferenceFrame.getWorldFrame());
      surfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

      surfacePose.getPosition().set(contactPointPosition);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, surfacePose.getOrientation());
      contactPointFrame.setPoseAndUpdate(surfacePose);
   }

   private void assertInitialized()
   {
      if (rigidBody == null)
      {
         throw new RuntimeException("Rigid body hasn't been set.");
      }
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
