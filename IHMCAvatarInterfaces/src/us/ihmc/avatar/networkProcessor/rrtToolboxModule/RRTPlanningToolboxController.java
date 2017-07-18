package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.RRTPlanningRequestPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class RRTPlanningToolboxController extends ToolboxController
{
   private int cntUpdate = 0;
   
   private final YoBoolean isDone = new YoBoolean("isDone", registry);   
   
   private final AtomicReference<RRTPlanningRequestPacket> latestRequestReference = new AtomicReference<RRTPlanningRequestPacket>(null);
   
   public RRTPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      
      isDone.set(false);
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + cntUpdate);
      
      if(cntUpdate == 100)
         isDone.set(true);      
      cntUpdate++;      
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      RRTPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
      {
         return false;
      }
      
      PrintTools.info("temp input "+request.tempInputValue);
         
      
      return true;
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
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
