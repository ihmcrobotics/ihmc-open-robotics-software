package us.ihmc.rdx.ui.teleoperation.locomotion;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.log.LogTools;

public class RDXPauseWalkingMode
{
   private final CommunicationHelper communicationHelper;
   private final PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();


   public RDXPauseWalkingMode(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;
   }

   public void setPauseWalking(boolean pauseWalking)
   {
      pauseWalkingMessage.setPause(pauseWalking);
      communicationHelper.publishToController(pauseWalkingMessage);

      if (pauseWalking)
         LogTools.info("Paused Walking");
      else
         LogTools.info("Continue Walking");
   }
}
