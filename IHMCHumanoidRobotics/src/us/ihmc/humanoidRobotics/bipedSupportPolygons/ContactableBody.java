package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface ContactableBody
{
   public abstract String getName();

   public abstract RigidBody getRigidBody();

   public abstract MovingReferenceFrame getFrameAfterParentJoint();

   public abstract int getTotalNumberOfContactPoints();

   public abstract List<FramePoint3D> getContactPointsCopy();
}
