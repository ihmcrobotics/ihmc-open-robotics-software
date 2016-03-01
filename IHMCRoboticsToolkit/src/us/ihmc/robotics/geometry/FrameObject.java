package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameObject<T> extends ReferenceFrameHolder, Transformable<T>
{
   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);

   public abstract void setToZero(ReferenceFrame referenceFrame);

   public abstract void setToNaN(ReferenceFrame referenceFrame);
}