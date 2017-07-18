package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.RRTPlanningRequestPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RRTPlanningToolboxController extends ToolboxController
{
   private final AtomicReference<RRTPlanningRequestPacket> latestRequestReference = new AtomicReference<RRTPlanningRequestPacket>(null);
   
   public RRTPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      // TODO Auto-generated constructor stub
   }

   @Override
   protected void updateInternal()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   protected boolean initialize()
   {
      RRTPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;
      
      
      return true;
   }

   @Override
   protected boolean isDone()
   {
      return false;
   }

   public PacketConsumer<RRTPlanningRequestPacket> createRequestConsumer()
   {
      return new PacketConsumer<RRTPlanningRequestPacket>()
      {
         @Override
         public void receivedPacket(RRTPlanningRequestPacket packet)
         {
            if (packet == null)
               return;
            latestRequestReference.set(packet);
         }
      };
   }

}
