package us.ihmc.avatar.networkProcessor.poseValidityToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.PoseValidityRequestPacket;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class PoseValidityToolboxController extends ToolboxController
{
   private final AtomicReference<PoseValidityRequestPacket> latestRequestReference = new AtomicReference<PoseValidityRequestPacket>(null);
   
   public PoseValidityToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);      
      PrintTools.info("PoseValdityToolboxController Initiated");
   }
   
   public PacketConsumer<PoseValidityRequestPacket> createRequestConsumer()
   {
      PrintTools.info("PoseValdityToolboxController createRequestConsumer");
      return new PacketConsumer<PoseValidityRequestPacket>()
      {         
         @Override
         public void receivedPacket(PoseValidityRequestPacket packet)
         {
            PrintTools.info("receivedPacket");
            if (packet == null)
               return;
            PrintTools.info("receivedPacket");
            latestRequestReference.set(packet);
         }
      };
   }
   

   @Override
   protected void updateInternal()
   {
      PrintTools.info("pose toolbox update");
      // TODO Auto-generated method stub
      
   }

   @Override
   protected boolean initialize()
   {
      PrintTools.info("PoseValdityToolboxController initialize");
      return false;
   }

   @Override
   protected boolean isDone()
   {
      PrintTools.info("PoseValdityToolboxController isDone");
      return false;
   }

}
