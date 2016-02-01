package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
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

   public void get(FramePoint positionToPack)
   {
      configuration.getPositionIncludingFrame(positionToPack);
   }

   public void get(FrameOrientation orientationToPack)
   {
      configuration.getOrientationIncludingFrame(orientationToPack);
   }

   public void set(FramePose newPose)
   {
      configuration.setPoseIncludingFrame(newPose);
   }

}
