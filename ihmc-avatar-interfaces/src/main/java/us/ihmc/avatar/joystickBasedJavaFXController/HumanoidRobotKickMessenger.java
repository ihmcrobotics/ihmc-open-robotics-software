package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public interface HumanoidRobotKickMessenger
{
   void sendFlamingoHomeStance(RobotSide robotSide, double trajectoryDuration, double stanceWidth, SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames);

   void sendKick(RobotSide robotSide, double trajectoryDuration, double stanceWidth, SegmentDependentList<RobotSide, ? extends ReferenceFrame> segmentDependentList);

   void sendPutFootDown(RobotSide robotSide, double trajectoryDuration, double stanceWidth, SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames);
}
