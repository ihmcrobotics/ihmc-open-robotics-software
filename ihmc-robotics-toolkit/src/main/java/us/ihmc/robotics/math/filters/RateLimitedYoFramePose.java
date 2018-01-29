package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoFramePose extends YoFramePoseUsingQuaternions
{
   private final RateLimitedYoFrameVector position;
   private final RateLimitedYoFrameOrientation orientation;

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                 YoFramePose rawPose)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, rawPose, rawPose.getReferenceFrame());
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                 ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, maxRate, dt, null, referenceFrame);
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, YoFramePose rawPose)
   {
      this(namePrefix, nameSuffix, registry, null, dt, rawPose, rawPose.getReferenceFrame());
      setMaxRate(maxRate);
   }

   public RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoVariableRegistry registry, double maxRate, double dt, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, null, dt, null, referenceFrame);
      setMaxRate(maxRate);
   }

   private RateLimitedYoFramePose(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maxRate, double dt,
                                  YoFramePose rawPose, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.position = new RateLimitedYoFrameVector(namePrefix, nameSuffix, registry, maxRate, dt, rawPose.getPosition());
      this.orientation = new RateLimitedYoFrameOrientation(namePrefix, nameSuffix, registry, maxRate, dt, rawPose.getOrientation());

      reset();
   }

   public void setMaxRate(double maxRate)
   {
      this.position.setMaxRate(maxRate);
      this.orientation.setMaxRate(maxRate);
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
   }

   public void update(YoFramePoseUsingQuaternions yoFramePoseUnfiltered)
   {
      checkReferenceFrameMatch(yoFramePoseUnfiltered);
      position.update(yoFramePoseUnfiltered.getPosition());
      orientation.update(yoFramePoseUnfiltered.getOrientation());
   }

   public void update(FramePose3D framePoseUnfiltered)
   {
      checkReferenceFrameMatch(framePoseUnfiltered);
      position.update(framePoseUnfiltered.getPosition());
      orientation.update(framePoseUnfiltered.getOrientation());
   }
}
