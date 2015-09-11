package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.trajectories.providers.PositionProvider;


public class YoPositionProvider implements PositionProvider
{
   private final YoFramePoint framePoint;

   public YoPositionProvider(YoFramePoint yoFramePoint)
   {
      this.framePoint = yoFramePoint;
   }

   public void get(FramePoint positionToPack)
   {
      framePoint.getFrameTupleIncludingFrame(positionToPack);
   }

   public void set(FramePoint framePoint)
   {
      this.framePoint.set(framePoint);
   }
}
