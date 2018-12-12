package us.ihmc.robotics.contactable;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface ContactableBody
{
   public abstract String getName();

   public abstract RigidBodyBasics getRigidBody();

   public abstract MovingReferenceFrame getFrameAfterParentJoint();

   public abstract int getTotalNumberOfContactPoints();

   public abstract List<FramePoint3D> getContactPointsCopy();
}
