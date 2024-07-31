package us.ihmc.rdx.ui.remoteCaptury;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXCapturyManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean capturyEnabled = new ImBoolean(false);
   private final ImBoolean actorDeleted = new ImBoolean(false);
   private final ImBoolean snapActor = new ImBoolean(false);
   public void renderMenuBar()
   {
      ImGui.setNextWindowSize(350.0f, 250.0f);
      if (imgui.internal.ImGui.beginMenu(labels.get("Captury")))
      {
         ImGuiTools.separatorText("Controls");
         renderEnableCheckbox();

         ImGuiTools.separatorText("Status");

         imgui.internal.ImGui.endMenu();
      }
   }

   public void renderEnableCheckbox()
   {
      if (imgui.internal.ImGui.menuItem(labels.get("Remote Captury Enabled"), "", capturyEnabled))
      {
         if (capturyEnabled.get())
         {
            RDXBaseUI.pushNotification("Enabling Remote Captury...");
            RDXBaseUI.pushNotification("Remote Captury enabled");
         }
         else
         {
            RDXBaseUI.pushNotification("Remote Captury disabled");
         }
      }
      if (imgui.internal.ImGui.isItemHovered())
      {
         imgui.internal.ImGui.setTooltip("Be ready to connect to Captury Live before turning on.");
      }
      if (imgui.internal.ImGui.menuItem(labels.get("Delete Actor"), "", actorDeleted))
      {
         if(actorDeleted.get())
         {
            RDXBaseUI.pushNotification("Deleting Current Actor...");
         }
      }
      if (imgui.internal.ImGui.menuItem(labels.get("Snap new Actor"), "", snapActor))
      {
         if(snapActor.get())
         {
            RDXBaseUI.pushNotification("Snapping Actor...");
         }
      }
   }

   public ImBoolean getCapturyEnabled()
   {
      return capturyEnabled;
   }

   public ImBoolean getActorDeleted()
   {
      return actorDeleted;
   }

   public ImBoolean getSnapActor()
   {
      return snapActor;
   }
   public void setActorDeleted(boolean actorDeleted)
   {
      this.actorDeleted.set(actorDeleted);
   }
   public void setSnapActor(boolean snapActor)
   {
      this.snapActor.set(snapActor);
   }
}
