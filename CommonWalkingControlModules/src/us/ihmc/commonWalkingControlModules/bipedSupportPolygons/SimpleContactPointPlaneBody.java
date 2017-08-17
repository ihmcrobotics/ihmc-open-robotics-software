package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * Simple implementation of {@link #ContactablePlaneBody} for a single contact point.
 *
 * @author Georg
 *
 */
public class SimpleContactPointPlaneBody implements ContactablePlaneBody
{
   private final String name;
   private final RigidBody rigidBody;
   private final PoseReferenceFrame contactFrame;

   private final List<FramePoint2D> contactPoints = new ArrayList<>();

   public SimpleContactPointPlaneBody(String name, RigidBody rigidBody, RigidBodyTransform contactFramePoseInJointFrame)
   {
      this.name = name;
      this.rigidBody = rigidBody;

      ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
      contactFrame = new PoseReferenceFrame(name + "Frame", frameAfterJoint);
      contactFrame.setPoseAndUpdate(contactFramePoseInJointFrame);
      contactPoints.add(new FramePoint2D(contactFrame));
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public MovingReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   @Override
   public int getTotalNumberOfContactPoints()
   {
      return 1;
   }

   @Override
   public List<FramePoint3D> getContactPointsCopy()
   {
      List<FramePoint3D> contactPoints = new ArrayList<>();
      contactPoints.add(new FramePoint3D(contactFrame));
      return contactPoints;
   }

   @Override
   public ReferenceFrame getSoleFrame()
   {
      return contactFrame;
   }

   @Override
   public List<FramePoint2D> getContactPoints2d()
   {
      return contactPoints;
   }

   @Override
   public void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform)
   {
      contactFrame.setPoseAndUpdate(transform);
   }

}
