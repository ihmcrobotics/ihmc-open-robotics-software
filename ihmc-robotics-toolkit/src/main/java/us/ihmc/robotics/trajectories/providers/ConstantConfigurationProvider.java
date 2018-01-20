package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ConstantConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose3D configuration;

   public ConstantConfigurationProvider(ReferenceFrame referenceFrame)
   {
      this.configuration = new FramePose3D(referenceFrame);
   }

   public ConstantConfigurationProvider(FramePoint3D framePoint)
   {
      configuration = new FramePose3D(framePoint, new FrameQuaternion(framePoint.getReferenceFrame()));
   }

   public ConstantConfigurationProvider(FramePose3D framePose)
   {
      this.configuration = new FramePose3D(framePose);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(configuration.getPosition());
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(configuration.getOrientation());
   }
}
