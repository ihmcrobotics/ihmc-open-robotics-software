package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

import java.util.List;

public interface ContactablePlaneBody
{
   public abstract String getName();

   public abstract RigidBody getRigidBody();

   public abstract ReferenceFrame getFrameAfterParentJoint();

   public abstract ReferenceFrame getSoleFrame();

   public abstract int getTotalNumberOfContactPoints();

   public abstract List<FramePoint> getContactPointsCopy();

   public abstract List<FramePoint2d> getContactPoints2d();
}
