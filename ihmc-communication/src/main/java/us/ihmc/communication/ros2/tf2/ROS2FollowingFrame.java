package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ros2.ROS2Node;

/**
 * ROS 2 frame that follows another reference frame in root.
 */
public class ROS2FollowingFrame extends ROS2Frame
{
   private final ReferenceFrame frameToFollow;

   public ROS2FollowingFrame(ROS2Node ros2Node, String id, ReferenceFrame parentFrame, ReferenceFrame frameToFollow)
   {
      super(ros2Node, id, parentFrame, null, parentFrame.isAStationaryFrame() && frameToFollow.isAStationaryFrame(), frameToFollow.isZupFrame(), false);

      this.frameToFollow = frameToFollow;
   }

   @Override
   protected void onNewTransformReceived(TransformStamped newTransform)
   {
      // Do nothing
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      frameToFollow.getTransformToDesiredFrame(transformToParent, getParent());
      publishTFMessages();
   }
}
