package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.robotics.geometry.FramePose;

/**
* @author twan
*         Date: 5/27/13
*/
public class ChangeableConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose configuration;

   public ChangeableConfigurationProvider()
   {
      configuration = new FramePose();
   }

   public ChangeableConfigurationProvider(FramePose initialConfiguration)
   {
      configuration = new FramePose(initialConfiguration);
   }

   public void get(FramePose framePose)
   {
      framePose.setPoseIncludingFrame(configuration);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      configuration.getPositionIncludingFrame(positionToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      configuration.getOrientationIncludingFrame(orientationToPack);
   }

   public void set(FramePose newPose)
   {
      configuration.setPoseIncludingFrame(newPose);
   }

}
