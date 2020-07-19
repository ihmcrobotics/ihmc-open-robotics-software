package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;


public class YoPositionProvider implements PositionProvider
{
   private final YoFramePoint3D framePoint;

   public YoPositionProvider(YoFramePoint3D yoFramePoint)
   {
      this.framePoint = yoFramePoint;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(framePoint);
   }

   public void set(FramePoint3D framePoint)
   {
      this.framePoint.set(framePoint);
   }
}
