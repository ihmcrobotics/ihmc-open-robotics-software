package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameObject extends ReferenceFrameHolder
{
   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);

   public abstract void applyTransform(RigidBodyTransform transform);
}