package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;

/**
* @author twan
*         Date: 5/27/13
*/
public class ChangeableConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose3D configuration;

   public ChangeableConfigurationProvider()
   {
      configuration = new FramePose3D();
   }

   public ChangeableConfigurationProvider(FramePose3D initialConfiguration)
   {
      configuration = new FramePose3D(initialConfiguration);
   }

   public void set(FramePose3DReadOnly newPose)
   {
      configuration.setIncludingFrame(newPose);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return configuration.getReferenceFrame();
   }

   @Override
   public FramePose3DReadOnly getPose()
   {
      return configuration;
   }

}
