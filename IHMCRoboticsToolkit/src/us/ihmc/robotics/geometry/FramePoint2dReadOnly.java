package us.ihmc.robotics.geometry;

import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FramePoint2dReadOnly
{
   public ReferenceFrame getReferenceFrame();
   public void get(Tuple2d tuple2dToPack);
   public void get(Tuple3d tuple3dToPack);
   public double getX();
   public double getY();
}
