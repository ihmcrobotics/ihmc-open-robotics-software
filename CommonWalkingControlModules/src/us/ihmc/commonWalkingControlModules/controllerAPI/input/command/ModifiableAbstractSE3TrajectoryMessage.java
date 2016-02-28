package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ModifiableAbstractSE3TrajectoryMessage<CorrespondingMessageType extends AbstractSE3TrajectoryMessage<CorrespondingMessageType>> extends FrameSE3TrajectoryPointList
{

   public ModifiableAbstractSE3TrajectoryMessage()
   {
   }

   public void set(CorrespondingMessageType trajectoryMessage)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), trajectoryMessage);
   }
}
