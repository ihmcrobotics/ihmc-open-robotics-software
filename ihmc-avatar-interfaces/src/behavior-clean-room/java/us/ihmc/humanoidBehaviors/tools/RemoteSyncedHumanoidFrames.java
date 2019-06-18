package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.ros2.Ros2Node;

import java.util.function.Function;

public class RemoteSyncedHumanoidFrames extends RemoteSyncedRobotModel
{
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final FramePose3D temporaryPoseForQuickReading = new FramePose3D();

   public RemoteSyncedHumanoidFrames(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      super(robotModel, ros2Node);

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
   }

   public HumanoidReferenceFrames pollHumanoidReferenceFrames()
   {
      pollFullRobotModel();

      humanoidReferenceFrames.updateFrames();

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

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      temporaryPoseForQuickReading.setFromReferenceFrame(frameSelector.apply(pollHumanoidReferenceFrames()));
      return temporaryPoseForQuickReading;
   }
}
