package us.ihmc.robotics.math.filters;



import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class AccelerationLimitedYoFrameVector2d extends YoFrameVector2D
{
   private final AccelerationLimitedYoVariable x, y;

   private AccelerationLimitedYoFrameVector2d(AccelerationLimitedYoVariable x, AccelerationLimitedYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static AccelerationLimitedYoFrameVector2d createAccelerationLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                             DoubleProvider maxRate, DoubleProvider maxAcceleration, double dt, ReferenceFrame referenceFrame)
   {
      AccelerationLimitedYoVariable x = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, dt);
      AccelerationLimitedYoVariable y = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, dt);

      AccelerationLimitedYoFrameVector2d ret = new AccelerationLimitedYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }


   public static AccelerationLimitedYoFrameVector2d createAccelerationLimitedYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                             DoubleProvider maxRate, DoubleProvider maxAcceleration, double dt, YoFrameVector2D unfilteredVector)
   {
      AccelerationLimitedYoVariable x = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, unfilteredVector.getYoX(), dt);
      AccelerationLimitedYoVariable y = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, unfilteredVector.getYoY(), dt);

      AccelerationLimitedYoFrameVector2d ret = new AccelerationLimitedYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

      return ret;
   }

   public void setGainsByPolePlacement(double w0, double zeta)
   {
      x.setGainsByPolePlacement(w0, zeta);
      y.setGainsByPolePlacement(w0, zeta);
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

   public void update(Vector2D vector2dUnfiltered)
   {
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void update(FrameVector2D vector2dUnfiltered)
   {
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}


