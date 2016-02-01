package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   public void get(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      this.orientation.set(orientation);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return orientation.getReferenceFrame();
   }
}
