package us.ihmc.robotics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose2DBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFramePose2D implements FixedFramePose2DBasics
{
   private final ReferenceFrame referenceFrame;
   private final YoFramePoint2D position;
   private final YoFrameOrientation2D orientation;

   public YoFramePose2D(String prefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this.referenceFrame = referenceFrame;
      position = new YoFramePoint2D(prefix, referenceFrame, registry);
      orientation = new YoFrameOrientation2D(prefix, referenceFrame, registry);
   }

   @Override
   public FixedFrameOrientation2DBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public FixedFramePoint2DBasics getPosition()
   {
      return position;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
