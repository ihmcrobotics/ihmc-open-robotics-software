package us.ihmc.robotics.math.filters;


import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class RateLimitedYoFrameVector2d extends YoFrameVector2d
{
   private final RateLimitedYoVariable x, y;

   private RateLimitedYoFrameVector2d(RateLimitedYoVariable x, RateLimitedYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static RateLimitedYoFrameVector2d createRateLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, dt);

      RateLimitedYoFrameVector2d ret = new RateLimitedYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }

   public static RateLimitedYoFrameVector2d createRateLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           YoDouble maxRate, double dt, ReferenceFrame referenceFrame)
   {
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, dt);

      RateLimitedYoFrameVector2d ret = new RateLimitedYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }


   public static RateLimitedYoFrameVector2d createRateLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           double maxRate, double dt, YoFrameVector2d unfilteredVector)
   {
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoX(), dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoY(), dt);

      RateLimitedYoFrameVector2d ret = new RateLimitedYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

      return ret;
   }


   public static RateLimitedYoFrameVector2d createRateLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           YoDouble maxRate, double dt, YoFrameVector2d unfilteredVector)
   {
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoX(), dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoY(), dt);

      RateLimitedYoFrameVector2d ret = new RateLimitedYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

      return ret;
   }

   public void update()
   {
      x.update();
      y.update();
   }

   public void update(double xUnfiltered, double yUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
   }

   public void update(Vector2DReadOnly vector2dUnfiltered)
   {
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void update(FrameTuple2DReadOnly vector2dUnfiltered)
   {
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void setMaxRate(double maxRate)
   {
      x.setMaxRate(maxRate);
      y.setMaxRate(maxRate);
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}

