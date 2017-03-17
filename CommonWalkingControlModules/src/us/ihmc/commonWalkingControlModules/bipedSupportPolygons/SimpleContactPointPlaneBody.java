package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
   private final ReferenceFrame contactFrame;

   private final List<FramePoint2d> contactPoints = new ArrayList<>();

   public SimpleContactPointPlaneBody(String name, RigidBody rigidBody, Point3D contactPointInBodyFrame)
   {
      this.name = name;
      this.rigidBody = rigidBody;

      RigidBodyTransform bodyToContact = new RigidBodyTransform();
      bodyToContact.setTranslation(contactPointInBodyFrame);
      contactFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent(name + "Frame", rigidBody.getBodyFixedFrame(), bodyToContact);

      contactPoints.add(new FramePoint2d(contactFrame));
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
   public ReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   @Override
   public int getTotalNumberOfContactPoints()
   {
      return 1;
   }

   @Override
   public List<FramePoint> getContactPointsCopy()
   {
      List<FramePoint> contactPoints = new ArrayList<>();
      contactPoints.add(new FramePoint(contactFrame));
      return contactPoints;
   }

   @Override
   public ReferenceFrame getSoleFrame()
   {
      return contactFrame;
   }

   @Override
   public List<FramePoint2d> getContactPoints2d()
   {
      return contactPoints;
   }

}
