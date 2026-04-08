package us.ihmc.robotics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.yoVariables.euclid.YoOrientation2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFrameOrientation2D implements FixedFrameOrientation2DBasics
{
   private final YoOrientation2D orientation;
   private final ReferenceFrame referenceFrame;

   public YoFrameOrientation2D(String prefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this.referenceFrame = referenceFrame;
      orientation = new YoOrientation2D(prefix, registry);
   }

   @Override
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      orientation.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.applyInverseTransform(transform);
   }

   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
