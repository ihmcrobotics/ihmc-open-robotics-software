package us.ihmc.rdx.ui.remoteCaptury;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.ArrayList;
import java.util.List;

public class RDXCapturyManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean capturyEnabled = new ImBoolean(false);
   private final ImBoolean actorDeleted = new ImBoolean(false);
   private final ImBoolean hideFingers = new ImBoolean(false);
   private final ImBoolean hideAll = new ImBoolean(false);
   private final ImBoolean snapActor = new ImBoolean(false);
   private List<Integer> actorArray = new ArrayList<>();
   private int selectedIndex = 0;
   private int selectedActorID = 0;
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
      if(actorArray.size() > 0)
      {
         if (ImGui.beginCombo(labels.get("Select Actor"), String.valueOf(actorArray.get(selectedIndex))))
         {
            for (int i = 0; i < actorArray.size(); i++)
            {
               int actorID = actorArray.get(i);
               if (ImGui.selectable(String.valueOf(actorID), selectedIndex == i))
               {
                  selectedIndex = i;
                  selectedActorID = actorID;
               }
            }
            ImGui.endCombo();
         }
      }


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
      if (imgui.internal.ImGui.menuItem(labels.get("Hide Fingers"), "", hideFingers))
      {
         if(hideFingers.get())
         {
            RDXBaseUI.pushNotification("Hidding Fingers...");
         }
      }
      if (imgui.internal.ImGui.menuItem(labels.get("Hide All Frames"), "", hideAll))
      {
         if(hideAll.get())
         {
            RDXBaseUI.pushNotification("Hidding All Frames...");
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
   public ImBoolean getHideFingers()
   {
      return hideFingers;
   }
   public ImBoolean getHideAll()
   {
      return hideAll;
   }
   public void setSnapActor(boolean snapActor)
   {
      this.snapActor.set(snapActor);
   }

   public void addToActorArray(int Actor_ID)
   {
      actorArray.add(Actor_ID);

   }

   public int getSelectedActorID()
   {
      return selectedActorID;
   }
   public boolean checkContain(int Actor_ID)
   {
      return actorArray.contains(Actor_ID);
   }
}
