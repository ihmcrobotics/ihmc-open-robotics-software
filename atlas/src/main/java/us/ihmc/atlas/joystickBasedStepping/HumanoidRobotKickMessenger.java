package us.ihmc.atlas.joystickBasedStepping;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public interface HumanoidRobotKickMessenger
{
   void sendFlamingoHomeStance(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                               SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames);

   void sendKick(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                 SegmentDependentList<RobotSide, ? extends ReferenceFrame> segmentDependentList);

   void sendPutFootDown(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                        SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames);
}
