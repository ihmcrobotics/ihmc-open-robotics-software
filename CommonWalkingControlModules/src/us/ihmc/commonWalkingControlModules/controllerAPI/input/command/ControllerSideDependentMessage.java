package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ControllerSideDependentMessage<T extends ControllerSideDependentMessage<T, M>, M extends Packet<M>> extends ControllerMessage<T, M>
{
   public abstract RobotSide getRobotSide();
}
