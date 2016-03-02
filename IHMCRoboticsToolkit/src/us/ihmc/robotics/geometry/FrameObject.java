package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameObject<T> extends ReferenceFrameHolder, GeometryObject<T>
{
   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);

   public abstract void setToZero(ReferenceFrame referenceFrame);

   public abstract void setToNaN(ReferenceFrame referenceFrame);
}