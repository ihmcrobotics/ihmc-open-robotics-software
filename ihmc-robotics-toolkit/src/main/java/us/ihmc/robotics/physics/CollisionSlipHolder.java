package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;

public interface CollisionSlipHolder
{
   Collidable getCollidableA();

   Collidable getCollidableB();

   FrameVector3DBasics getEstimatedSlipFromBToA();
}
