package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.ConfirmableRequestMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;

import java.util.UUID;

/**
 * A freezable node that has a mechanism for the freeze
 * to end early if we know the change has propagated.
 */
public class Confirmable extends Freezable
{
   private static final MutableLong nextRequestID = new MutableLong();

   private final Notification needToSendRequest = new Notification();
   private final Notification needToSendConfirmation = new Notification();
   private final ROS2ActorDesignation actorDesignation;

   //   private UUID requestUUID;
   private long requestUUID;

   public Confirmable(ROS2ActorDesignation actorDesignation)
   {
      this.actorDesignation = actorDesignation;
   }

   @Override
   public void freeze()
   {
      super.freeze();

      if (!needToSendRequest.peek()) // Avoid sending multiple requests in one tick
      {
         needToSendRequest.set();

         //      requestUUID = UUID.randomUUID();
         requestUUID = nextRequestID.getAndIncrement();
         LogTools.info("Request: {}:{} Actor: {}", this.getClass().getSimpleName(), requestUUID, actorDesignation.name());
      }
   }

   public void toMessage(ConfirmableRequestMessage message)
   {
      if (needToSendRequest.poll())
      {
         message.setValue(ConfirmableRequestMessage.REQUEST);
//         MessageTools.toMessage(requestUUID, message.getRequestUuid());
         message.getRequestUuid().setLeastSignificantBits(requestUUID);
      }
      else if (needToSendConfirmation.poll())
      {
         message.setValue(ConfirmableRequestMessage.CONFIRMATION);
//         MessageTools.toMessage(requestUUID, message.getRequestUuid());
         message.getRequestUuid().setLeastSignificantBits(requestUUID);

         LogTools.info("Confirming: {}:{} Actor: {}", this.getClass().getSimpleName(), requestUUID, actorDesignation.name());
      }
      else
      {
         message.setValue(ConfirmableRequestMessage.NOOP);
      }
   }

   public void fromMessage(ConfirmableRequestMessage message)
   {
      if (message.getValue() == ConfirmableRequestMessage.REQUEST)
      {
         needToSendConfirmation.set();
//         requestUUID = MessageTools.toUUID(message.getRequestUuid());
         requestUUID = message.getRequestUuid().getLeastSignificantBits();
      }
      else if (message.getValue() == ConfirmableRequestMessage.CONFIRMATION)
      {
//         UUID confirmationsRequestUUID = MessageTools.toUUID(message.getRequestUuid());
//         if (confirmationsRequestUUID.compareTo(requestUUID) == 0)
         long confirmationsRequestUUID = message.getRequestUuid().getLeastSignificantBits();
         if (confirmationsRequestUUID == requestUUID)
         {
            LogTools.info("Confirmed: {}:{} Actor: {}", this.getClass().getSimpleName(), confirmationsRequestUUID, actorDesignation.name());
            unfreeze();
         }
         else
         {
            LogTools.error("The heck man");
         }
      }
   }
}
