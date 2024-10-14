package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class RateLimitedYoFrameVector2D extends YoFrameVector2D
{
   private final RateLimitedYoVariable x, y;

   public RateLimitedYoFrameVector2D(String namePrefix, YoRegistry registry,
                                     DoubleProvider maxRate, double dt, YoFrameVector2D unfilteredVector)
   {
      this(namePrefix, "", registry, maxRate, dt, unfilteredVector);
   }

   public RateLimitedYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry,
                                     DoubleProvider maxRate, double dt, YoFrameVector2D unfilteredVector)
   {
      this(new RateLimitedYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoX(), dt),
           new RateLimitedYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoY(), dt),
           unfilteredVector.getReferenceFrame());
   }

   private RateLimitedYoFrameVector2D(RateLimitedYoVariable x, RateLimitedYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public void update()
   {
      x.update();
      y.update();
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}

