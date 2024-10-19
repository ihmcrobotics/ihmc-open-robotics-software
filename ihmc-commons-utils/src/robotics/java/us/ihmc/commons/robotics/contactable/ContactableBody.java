package us.ihmc.commons.robotics.contactable;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface ContactableBody
{
   String getName();

   RigidBodyBasics getRigidBody();

   MovingReferenceFrame getFrameAfterParentJoint();

   int getTotalNumberOfContactPoints();

   List<FramePoint3D> getContactPointsCopy();
}
