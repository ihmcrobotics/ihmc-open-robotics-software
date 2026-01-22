package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * ROS 2 frame that follows another reference frame in root.
 */
public class ROS2FollowingFrame extends ROS2Frame
{
   private final ReferenceFrame frameToFollow;

   public ROS2FollowingFrame(String id, ReferenceFrame parentFrame, ReferenceFrame frameToFollow)
   {
      super(id, parentFrame, null, parentFrame.isAStationaryFrame() && frameToFollow.isAStationaryFrame(), frameToFollow.isZupFrame(), false);

      this.frameToFollow = frameToFollow;

      postConstruction();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      frameToFollow.getTransformToDesiredFrame(transformToParent, getParent());
      markUpdateTime();
      publishTFMessages();
   }
}
