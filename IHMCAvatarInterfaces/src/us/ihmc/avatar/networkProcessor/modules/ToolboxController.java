package us.ihmc.avatar.networkProcessor.modules;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;

public abstract class ToolboxController<T extends StatusPacket<T>>
{
   private final StatusMessageOutputManager statusOutputManager;
   private final int numberOfTicksToSendSolution;

   private boolean initialize = true;
   private PacketDestination packetDestination = null;
   private int tickCount = 0;

   public ToolboxController(StatusMessageOutputManager statusOutputManager)
   {
      this(statusOutputManager, 1);
   }

   public ToolboxController(StatusMessageOutputManager statusOutputManager, int numberOfTicksToSendSolution)
   {
      this.statusOutputManager = statusOutputManager;
      this.numberOfTicksToSendSolution = numberOfTicksToSendSolution;
   }

   public void requestInitialize()
   {
      initialize = true;
   }

   public void setPacketDestination(PacketDestination packetDestination)
   {
      this.packetDestination = packetDestination;
   }

   public void update()
   {
      if (initialize)
      {
         if (!initialize()) // Return until the initialization succeeds
            return;
         initialize = false;
      }

      T result = updateInternal();

      tickCount++;
      if (packetDestination != null && tickCount == numberOfTicksToSendSolution)
      {
         tickCount = 0;
         result.setDestination(packetDestination);
         statusOutputManager.reportStatusMessage(result);
      }
   }

   abstract protected T updateInternal();
   abstract protected boolean initialize();
}
