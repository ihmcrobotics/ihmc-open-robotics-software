package us.ihmc.humanoidRobotics.communication.walkingPreviewToolboxAPI;

import controller_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;

public class WalkingControllerPreviewInputCommand implements Command<WalkingControllerPreviewInputCommand, WalkingControllerPreviewInputMessage>
{
   private final FootstepDataListCommand foostepCommand = new FootstepDataListCommand();

   public WalkingControllerPreviewInputCommand()
   {
   }
   
   @Override
   public void clear()
   {
      foostepCommand.clear();
   }
   
   @Override
   public void setFromMessage(WalkingControllerPreviewInputMessage message)
   {
      foostepCommand.setFromMessage(message.getFootsteps());
   }

   @Override
   public void set(WalkingControllerPreviewInputCommand other)
   {
      foostepCommand.set(other.foostepCommand);
   }

   public FootstepDataListCommand getFoostepCommand()
   {
      return foostepCommand;
   }

   @Override
   public Class<WalkingControllerPreviewInputMessage> getMessageClass()
   {
      return WalkingControllerPreviewInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return foostepCommand.isCommandValid();
   }
}
