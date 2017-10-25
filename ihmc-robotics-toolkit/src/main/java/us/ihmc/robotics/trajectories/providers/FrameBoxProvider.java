package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.Axis;
import us.ihmc.robotics.geometry.shapes.FrameBox3d;

public interface FrameBoxProvider
{
   public abstract FrameBox3d getFrameBox3d();

   public abstract Axis getDirection();
}
