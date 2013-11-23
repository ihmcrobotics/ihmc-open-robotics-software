package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.utilities.net.ObjectCommunicator;

public class ROSiRobotCommandDispatcher implements Runnable
{
   private FingerStateProvider fingerStateProvider = new FingerStateProvider();

   private ROSiRobotCommunicator rosHandCommunicator;

   public ROSiRobotCommandDispatcher(ObjectCommunicator objectCommunicator, String rosHostIP)
   {
      objectCommunicator.attachListener(FingerStatePacket.class, fingerStateProvider);
      rosHandCommunicator = new ROSiRobotCommunicator("http://" + rosHostIP + ":11311");
   }

   @Override
   public void run()
   {
      if (fingerStateProvider.isNewFingerStateAvailable())
      {
         FingerStatePacket packet = fingerStateProvider.pullPacket();
         rosHandCommunicator.sendHandCommand(packet);
      }

   }



}
