package us.ihmc.avatar.networkProcessor.modules;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public abstract class ToolboxController
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StatusMessageOutputManager statusOutputManager;

   private boolean initialize = true;
   // TODO Figure out to do multiple destination. Would be useful for modules like the quad-tree.
   private PacketDestination packetDestination = null;

   public ToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      parentRegistry.addChild(registry);
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

      updateInternal();
   }

   protected <T extends StatusPacket<T>> void reportMessage(T statusMessage)
   {
      if (packetDestination == null)
         return;

      statusMessage.setDestination(packetDestination);
      statusOutputManager.reportStatusMessage(statusMessage);
   }

   abstract protected void updateInternal();
   abstract protected boolean initialize();
   abstract protected boolean isDone();
}
