package us.ihmc.rdx.ui.vr;

import us.ihmc.behaviors.sharedControl.AssistancePhase;

public class RDXVRAssistanceStatus
{
   private AssistancePhase assistancePhase;
   private RDXVRAssistanceMenuMode menuMode;

   public RDXVRAssistanceStatus(AssistancePhase assistancePhase, RDXVRAssistanceMenuMode menuMode)
   {
      this.assistancePhase = assistancePhase;
      this.menuMode = menuMode;
   }

   public void setAssistancePhase(AssistancePhase assistancePhase)
   {
      this.assistancePhase = assistancePhase;
   }

   public void setActiveMenu(RDXVRAssistanceMenuMode menuMode)
   {
      this.menuMode = menuMode;
   }

   public AssistancePhase getAssistancePhase()
   {
      return assistancePhase;
   }

   public RDXVRAssistanceMenuMode getActiveMenu()
   {
      return menuMode;
   }
}
