package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;


/**
 * @author twan
 *         Date: 5/30/13
 */
public class YoSE3ConfigurationProvider implements SE3ConfigurationProvider
{
   private final YoFramePoint3D position;
   private final YoFrameQuaternion orientation;

   public YoSE3ConfigurationProvider(String name, ReferenceFrame frame, YoVariableRegistry registry)
   {
      position = new YoFramePoint3D(name, frame, registry);
      orientation = new YoFrameQuaternion(name, frame, registry);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public void setPose(FramePose3D pose)
   {
      this.position.setMatchingFrame(pose.getPosition());
      this.orientation.setMatchingFrame(pose.getOrientation());
   }
}
