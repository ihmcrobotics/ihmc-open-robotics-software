package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ModifiableSO3TrajectoryMessage<CorrespondingMessageType extends AbstractSO3TrajectoryMessage<CorrespondingMessageType>> extends FrameSO3TrajectoryPointList
{
   public ModifiableSO3TrajectoryMessage()
   {
   }

   public void set(CorrespondingMessageType trajectoryMessage)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), trajectoryMessage);
   }
}
