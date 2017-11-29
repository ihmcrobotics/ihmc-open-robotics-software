package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class YoQuaternionProvider implements OrientationProvider
{
   private final YoFrameQuaternion orientation;

   public YoQuaternionProvider(YoFrameQuaternion orientation)
   {
      this.orientation = orientation;
   }

   public YoQuaternionProvider(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      orientation = new YoFrameQuaternion(namePrefix + "Orientation", referenceFrame, registry);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void setOrientation(FrameQuaternion orientation)
   {
      this.orientation.set(orientation);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return orientation.getReferenceFrame();
   }
}
