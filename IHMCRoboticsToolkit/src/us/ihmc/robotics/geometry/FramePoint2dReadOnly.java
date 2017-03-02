package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FramePoint2dReadOnly
{
   public ReferenceFrame getReferenceFrame();
   public void get(Tuple2DBasics tuple2dToPack);
   public void get(Tuple3DBasics tuple3dToPack);
   public double getX();
   public double getY();
}
