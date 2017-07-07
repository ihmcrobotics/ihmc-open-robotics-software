package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

public interface FramePoint2dReadOnly
{
   public ReferenceFrame getReferenceFrame();
   public void get(Tuple2DBasics tuple2dToPack);
   public double getX();
   public double getY();
}
