package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;


/**
 * @author twan
 *         Date: 5/30/13
 */
public class YoSE3ConfigurationProvider implements SE3ConfigurationProvider
{
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;

   private final FramePoint tempPoint = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

   public YoSE3ConfigurationProvider(String name, ReferenceFrame frame, YoVariableRegistry registry)
   {
      position = new YoFramePoint(name, frame, registry);
      orientation = new YoFrameQuaternion(name, frame, registry);
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void get(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void setPose(FramePose pose)
   {
      pose.getPositionIncludingFrame(tempPoint);
      tempPoint.changeFrame(this.position.getReferenceFrame());
      this.position.set(tempPoint);

      pose.getOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(this.orientation.getReferenceFrame());
      this.orientation.set(tempOrientation);
   }
}
