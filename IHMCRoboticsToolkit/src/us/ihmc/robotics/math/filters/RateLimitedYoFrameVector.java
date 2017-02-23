package us.ihmc.robotics.math.filters;


import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class RateLimitedYoFrameVector extends YoFrameVector
{
   private final RateLimitedYoVariable x, y, z;

   private RateLimitedYoFrameVector(RateLimitedYoVariable x, RateLimitedYoVariable y, RateLimitedYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static RateLimitedYoFrameVector createRateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
           ReferenceFrame referenceFrame)
   {
      // alpha is a double
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable z = new RateLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, dt);

      RateLimitedYoFrameVector ret = new RateLimitedYoFrameVector(x, y, z, referenceFrame);

      return ret;
   }

   public static RateLimitedYoFrameVector createRateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           DoubleYoVariable maxRate, double dt, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, dt);
      RateLimitedYoVariable z = new RateLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, dt);

      RateLimitedYoFrameVector ret = new RateLimitedYoFrameVector(x, y, z, referenceFrame);

      return ret;
   }


   public static RateLimitedYoFrameVector createRateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt,
           YoFrameVector unfilteredVector)
   {
      // alpha is a double
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoX(), dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoY(), dt);
      RateLimitedYoVariable z = new RateLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoZ(), dt);

      RateLimitedYoFrameVector ret = new RateLimitedYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());

      return ret;
   }


   public static RateLimitedYoFrameVector createRateLimitedYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           DoubleYoVariable maxRate, double dt, YoFrameVector unfilteredVector)
   {
      // alpha is a YoVariable
      RateLimitedYoVariable x = new RateLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoX(), dt);
      RateLimitedYoVariable y = new RateLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoY(), dt);
      RateLimitedYoVariable z = new RateLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, unfilteredVector.getYoZ(), dt);

      RateLimitedYoFrameVector ret = new RateLimitedYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());

      return ret;
   }

   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
      z.update(zUnfiltered);
   }

   public void update(Vector3D vectorUnfiltered)
   {
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void update(FrameVector vectorUnfiltered)
   {
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
