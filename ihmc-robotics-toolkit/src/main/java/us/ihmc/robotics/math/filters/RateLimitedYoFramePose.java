package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoFramePose implements FixedFramePose3DBasics
{
   private final RateLimitedYoFramePoint position;
   private final RateLimitedYoFrameQuaternion orientation;

   private static DoubleProvider createMaxRateYoDouble(String namePrefix, String nameSuffix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                 FramePose3DReadOnly rawPose)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPose, rawPose.getReferenceFrame());
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                 ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt, FramePose3DReadOnly rawPose)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, rawPose, rawPose.getReferenceFrame());
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, createMaxRateYoDouble(namePrefix, nameSuffix, maxRate, registry), dt, null, referenceFrame);
   }

   private RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maxRate, double dt,
                                  FramePose3DReadOnly rawPose, ReferenceFrame referenceFrame)
   {
      if (rawPose != null)
      {
         this.position = new RateLimitedYoFramePoint(namePrefix, "Position" + nameSuffix, registry, maxRate, dt, rawPose.getPosition());
         this.orientation = new RateLimitedYoFrameQuaternion(namePrefix, "Orientation" + nameSuffix, registry, maxRate, dt, rawPose.getOrientation());
      }
      else
      {
         this.position = new RateLimitedYoFramePoint(namePrefix, "Position" + nameSuffix, registry, maxRate, dt, referenceFrame);
         this.orientation = new RateLimitedYoFrameQuaternion(namePrefix, "Orientation" + nameSuffix, registry, maxRate, dt, referenceFrame);
      }

      reset();
   }

   public void reset()
   {
      position.reset();
      orientation.reset();
   }

   public void update()
   {
      position.update();
      orientation.update();

      set(position, orientation);
   }

   public void update(FramePose3DReadOnly framePoseUnfiltered)
   {
      checkReferenceFrameMatch(framePoseUnfiltered);
      position.update(framePoseUnfiltered.getPosition());
      orientation.update(framePoseUnfiltered.getOrientation());

      set(position, orientation);
   }

   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose3DString(this) + "-" + getReferenceFrame();
   }
}
