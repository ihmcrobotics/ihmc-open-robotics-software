package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.ros2.Ros2Node;

public class RemoteSyncedHumanoidFrames extends RemoteSyncedRobotModel
{
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   public RemoteSyncedHumanoidFrames(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      super(robotModel, ros2Node);

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
   }

   public HumanoidReferenceFrames pollHumanoidReferenceFrames()
   {
      pollFullRobotModel();

      humanoidReferenceFrames.updateFrames();

//      LogTools.debug("Root joint position updated: {}", fullRobotModel.getRootJoint().getJointPose().getPosition());
//
//      LogTools.debug("Chest frame translation to world: {}", getString(humanoidReferenceFrames.getChestFrame()));
//      LogTools.debug("Pelvis frame translation to world: {}", getString(humanoidReferenceFrames.getPelvisFrame()));
//      LogTools.debug("MidFeetZUp frame translation to world: {}", getString(humanoidReferenceFrames.getMidFeetZUpFrame()));
//      LogTools.debug("MidFootZUpGround frame translation to world: {}", getString(humanoidReferenceFrames.getMidFootZUpGroundFrame()));

      return humanoidReferenceFrames;
   }

   private String getString(MovingReferenceFrame frame)
   {
      FramePose3D framePose = new FramePose3D();
      framePose.setFromReferenceFrame(frame);
      return framePose.getX() + ", " + framePose.getY();
   }

   public HumanoidReferenceFrames getHumanoidReferenceFrames()
   {
      return humanoidReferenceFrames;
   }
}
