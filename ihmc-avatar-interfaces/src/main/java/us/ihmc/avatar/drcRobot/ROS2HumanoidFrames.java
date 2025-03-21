package us.ihmc.avatar.drcRobot;

import us.ihmc.communication.ros2.tf2.ROS2FollowingFrame;
import us.ihmc.communication.ros2.tf2.ROS2Frame;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.communication.ros2.tf2.ROS2StaticFrame;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

/**
 * Stores {@link ROS2Frame} copies of all reference frames stemming from the robot's root frame.
 * <p>
 * The {@code ROS2Frame}s publish their respective {@link tf2_msgs.msg.dds.TFMessage}s,
 * resulting in the robot's entire tf tree being published.
 * <p>
 * To read the ROS standards for reference frames, see:
 * <ul>
 *    <li><a href="https://ros.org/reps/rep-0120.html">Coordinate Frames for Humanoid Robots</a></li>
 *    <li><a href="https://ros.org/reps/rep-0105.html">Coordinate Frames for Mobile Platforms</a></li>
 * </ul>
 */
public class ROS2HumanoidFrames
{
   @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
   private static final SideDependentList<String> SIDE_PREFIXES = new SideDependentList<>("l_", "r_");

   private final ROS2Node ros2Node;

   private final HumanoidReferenceFrames humanoidFrames;
   private final FullHumanoidRobotModel fullRobotModel;

   // Standard ROS 2 humanoid robot frames
   private final ROS2Frame mapFrame;
   private final ROS2Frame odomFrame;
   private ROS2Frame baseLinkFrame; // Pelvis frame
   private final ROS2MutableFrame baseFootprintFrame; // mid-foot-z-up frame, but z = min(leftFootZ, rightFootZ)
   private ROS2Frame torsoFrame;
   private ROS2Frame gazeFrame;
   private final SideDependentList<ROS2Frame> wristFrames = new SideDependentList<>();
   private final SideDependentList<ROS2Frame> gripperFrames = new SideDependentList<>();
   private final SideDependentList<ROS2Frame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ROS2Frame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ROS2Frame> toeFrames = new SideDependentList<>();

   // Map of all reference frame for which ROS2Frame copies were made
   private final Map<ReferenceFrame, ROS2Frame> ros2FrameCopyMap = new HashMap<>();

   // Set of all ROS2Frames made in this class
   private final Set<ROS2Frame> allROS2Frames = new LinkedHashSet<>();

   public ROS2HumanoidFrames(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.ros2Node = ros2Node;
      humanoidFrames = syncedRobotModel.getReferenceFrames();
      fullRobotModel = syncedRobotModel.getFullRobotModel();

      // Map is the local base frame in which the robot should not drift, but the robot's pose need not be continuous
      mapFrame = new ROS2StaticFrame(ros2Node, "map", ReferenceFrameTools.getWorldFrame(), new RigidBodyTransform(), true, true);
      allROS2Frames.add(mapFrame);

      // Odom is the local base frame in which the robot's pose is continuous, and may be subject to drift
      odomFrame = new ROS2MutableFrame(ros2Node, "odom", mapFrame, new RigidBodyTransform(), true);
      allROS2Frames.add(odomFrame);

      // Fill reference frame tree with robot frames
      fillTree(fullRobotModel.getRootJoint().getFrameAfterJoint());

      // Add base_footprint frame
      baseFootprintFrame = new ROS2MutableFrame(ros2Node, "base_footprint", baseLinkFrame, computeBaseFootprintToBaseLinkTransform(new RigidBodyTransform()));
      allROS2Frames.add(baseFootprintFrame);

      // Add toe frames
      for (RobotSide side : RobotSide.values)
      {
         ROS2Frame toeFrame = new ROS2StaticFrame(ros2Node, SIDE_PREFIXES.get(side) + "toe", ankleFrames.get(side), new RigidBodyTransform());
         toeFrames.put(side, toeFrame);
         allROS2Frames.add(toeFrame);
      }
   }

   private void fillTree(ReferenceFrame start)
   {
      // Create a ROS 2 frame copy of this frame
      createFrameCopy(start);

      // Repeat for all its child frames
      int childrenCount = start.getNumberOfChildren();
      for (int i = 0; i < childrenCount; ++i)
         fillTree(start.getChild(i));
   }

   private void createFrameCopy(ReferenceFrame frameToCopy)
   {
      ROS2Frame ros2FrameCopy = null;

      /*
       * First, check whether the frame we're copying is specified by REP 120 (https://ros.org/reps/rep-0120.html)
       * If it is, we create it with the name specified by REP 120 and give it a sensible parent frame
       */
      if (frameToCopy.equals(fullRobotModel.getRootJoint().getFrameAfterJoint()))
      {  // base link == frame before root joint
         baseLinkFrame = new ROS2FollowingFrame(ros2Node, "base_link", odomFrame, frameToCopy);
         ros2FrameCopy = baseLinkFrame;
      }
      else if (frameToCopy.equals(humanoidFrames.getChestFrame()))
      {  // torso == chest
         ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), baseLinkFrame);
         torsoFrame = new ROS2FollowingFrame(ros2Node, "torso", parentFrame, frameToCopy);
         ros2FrameCopy = torsoFrame;
      }
      else if (frameToCopy.equals(humanoidFrames.getHeadFrame()))
      {  // gaze == head
         ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), torsoFrame);
         gazeFrame = new ROS2FollowingFrame(ros2Node, "gaze", parentFrame, frameToCopy);
         ros2FrameCopy = gazeFrame;
      }
      else
      {
         for (RobotSide side : RobotSide.values)
         {
            if (ros2FrameCopy != null)
               break;

            String sidePrefix = SIDE_PREFIXES.get(side);

            if (frameToCopy.equals(fullRobotModel.getHand(side).getParentJoint().getFrameAfterJoint()))
            {  // wrist == parent joint frame of hand
               ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), torsoFrame);
               ROS2Frame wristFrame = new ROS2FollowingFrame(ros2Node, sidePrefix + "wrist", parentFrame, frameToCopy);
               wristFrames.put(side, wristFrame);
               ros2FrameCopy = wristFrame;
            }
            else if (frameToCopy.equals(fullRobotModel.getHandControlFrame(side)))
            {  // gripper == hand control frame
               ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), wristFrames.get(side));
               ROS2Frame gripperFrame = new ROS2FollowingFrame(ros2Node, sidePrefix + "gripper", parentFrame, frameToCopy);
               gripperFrames.put(side, gripperFrame);
               ros2FrameCopy = gripperFrame;
            }
            else if (frameToCopy.equals(humanoidFrames.getFootFrame(side)))
            {  // ankle == foot frame
               ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), baseLinkFrame);
               ROS2Frame ankleFrame = new ROS2FollowingFrame(ros2Node, sidePrefix + "ankle", parentFrame, frameToCopy);
               ankleFrames.put(side, ankleFrame);
               ros2FrameCopy = ankleFrame;
            }
            else if (frameToCopy.equals(humanoidFrames.getSoleFrame(side)))
            {  // sole == sole 👍
               ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), ankleFrames.get(side));
               ROS2Frame soleFrame = new ROS2FollowingFrame(ros2Node, sidePrefix + "sole", parentFrame, frameToCopy);
               soleFrames.put(side, soleFrame);
               ros2FrameCopy = soleFrame;
            }
         }
      }

      // No ROS 2 specific frame -- default to ros2_frameName
      if (ros2FrameCopy == null)
      {
         ROS2Frame parentFrame = ros2FrameCopyMap.getOrDefault(frameToCopy.getParent(), baseLinkFrame);
         ros2FrameCopy = new ROS2FollowingFrame(ros2Node, "ros2_" + frameToCopy.getName(), parentFrame, frameToCopy);
      }

      ros2FrameCopyMap.put(frameToCopy, ros2FrameCopy);
      allROS2Frames.add(ros2FrameCopy);
   }

   private RigidBodyTransform computeBaseFootprintToBaseLinkTransform(RigidBodyTransform transformToPack)
   {
      humanoidFrames.getMidFootZUpGroundFrame().getTransformToDesiredFrame(transformToPack, baseLinkFrame);
      Orientation3DBasics rotation = transformToPack.getRotation();
      rotation.setYawPitchRoll(0.0, rotation.getPitch(), rotation.getRoll());
      return transformToPack;
   }

   /**
    * Update all {@code ROS2Frames}. The frames will publish the tf messages.
    */
   public void update()
   {
      baseFootprintFrame.setNewTransformToParent(this::computeBaseFootprintToBaseLinkTransform);
      allROS2Frames.forEach(ReferenceFrame::update);
   }

   public ROS2Frame getMapFrame()
   {
      return mapFrame;
   }

   public ROS2Frame getOdomFrame()
   {
      return odomFrame;
   }

   public ROS2Frame getBaseLinkFrame()
   {
      return baseLinkFrame;
   }

   public ROS2Frame getBaseFootprintFrame()
   {
      return baseFootprintFrame;
   }

   public ROS2Frame getTorsoFrame()
   {
      return torsoFrame;
   }

   public ROS2Frame getGazeFrame()
   {
      return gazeFrame;
   }

   public ROS2Frame getWristFrame(RobotSide side)
   {
      return getWristFrames().get(side);
   }

   public SideDependentList<ROS2Frame> getWristFrames()
   {
      return wristFrames;
   }

   public ROS2Frame getGripperFrame(RobotSide side)
   {
      return getGripperFrames().get(side);
   }

   public SideDependentList<ROS2Frame> getGripperFrames()
   {
      return gripperFrames;
   }

   public ROS2Frame getAnkleFrame(RobotSide side)
   {
      return getAnkleFrames().get(side);
   }

   public SideDependentList<ROS2Frame> getAnkleFrames()
   {
      return ankleFrames;
   }

   public ROS2Frame getSoleFrame(RobotSide side)
   {
      return getSoleFrames().get(side);
   }

   public SideDependentList<ROS2Frame> getSoleFrames()
   {
      return soleFrames;
   }

   public ROS2Frame getToeFrame(RobotSide side)
   {
      return getToeFrames().get(side);
   }

   public SideDependentList<ROS2Frame> getToeFrames()
   {
      return toeFrames;
   }

   public ROS2Frame getROS2FrameCopy(ReferenceFrame referenceFrame)
   {
      return ros2FrameCopyMap.get(referenceFrame);
   }

   public void remove()
   {
      allROS2Frames.forEach(ReferenceFrame::remove);
   }
}
