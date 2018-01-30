package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RateLimitedYoFramePose extends YoFramePose
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

      if (rawPose != null)
      {
         this.position = new RateLimitedYoFrameVector(namePrefix, "Position" + nameSuffix, registry, maxRate, dt, rawPose.getPosition());
         this.orientation = new RateLimitedYoFrameOrientation(namePrefix, "Orientation" + nameSuffix, registry, maxRate, dt, rawPose.getOrientation());
      }
      else
      {
         this.position = new RateLimitedYoFrameVector(namePrefix, "Position" + nameSuffix, registry, maxRate, dt, referenceFrame);
         this.orientation = new RateLimitedYoFrameOrientation(namePrefix, "Orientation" + nameSuffix, registry, maxRate, dt, referenceFrame);
      }


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

      setPosition(position);
      setOrientation(orientation.getFrameOrientation());
   }

   public void update(FramePose3DReadOnly framePoseUnfiltered)
   {
      checkReferenceFrameMatch(framePoseUnfiltered);
      position.update(framePoseUnfiltered.getPosition());
      orientation.update(framePoseUnfiltered.getOrientation());

      setPosition(position);
      setOrientation(orientation.getFrameOrientation());
   }
}
