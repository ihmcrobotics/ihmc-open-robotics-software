package us.ihmc.commonWalkingControlModules.packetProviders;

import us.ihmc.communication.packets.manipulation.ControlStatusPacket;
import us.ihmc.communication.packets.manipulation.ControlStatusPacket.ControlStatus;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;

public class NetworkControlStatusProducer implements ControlStatusProducer
{
   
   private final GlobalDataProducer globalDataProducer;
   
   public NetworkControlStatusProducer(GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public void notifyHandTrajectoryInfeasible(RobotSide robotSide)
   {
      globalDataProducer.queueDataToSend(new ControlStatusPacket(robotSide, ControlStatus.HAND_TRAJECTORY_INFEASIBLE));
   }

}
