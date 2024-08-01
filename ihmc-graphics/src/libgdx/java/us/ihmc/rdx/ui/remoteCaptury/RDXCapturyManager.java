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
   private final List<Integer> actorArray = new ArrayList<>();
   private int selectedIndex = 0;
   private int selectedActorID = 0;
   private String ipAddress;
   private int port;

   public void renderMenuBar()
   {
      ImGui.setNextWindowSize(350.0f, 250.0f);
      if (ImGui.beginMenu(labels.get("Captury")))
      {
         ImGuiTools.separatorText("Controls");
         renderEnableCheckbox();

         ImGuiTools.separatorText("Status");
         ImGui.text("Connected Computer IP: " + (capturyEnabled.get() ? ipAddress : "None"));
         ImGui.text("Connected Computer Port: " + (capturyEnabled.get() ? port : "None"));
         ImGui.endMenu();
      }
   }

   public void renderEnableCheckbox()
   {
      if (!actorArray.isEmpty())
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

      if (ImGui.menuItem(labels.get("Remote Captury Enabled"), "", capturyEnabled))
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
      if (capturyEnabled.get())
      {
         if (ImGui.menuItem(labels.get("Delete Actor"), "", actorDeleted))
         {
            if (actorDeleted.get())
            {
               RDXBaseUI.pushNotification("Deleting Current Actor...");
            }
         }
         if (ImGui.menuItem(labels.get("Snap new Actor"), "", snapActor))
         {
            if (snapActor.get())
            {
               RDXBaseUI.pushNotification("Snapping Actor...");
            }
         }
         if (ImGui.menuItem(labels.get("Hide Fingers"), "", hideFingers))
         {
            if (hideFingers.get())
            {
               RDXBaseUI.pushNotification("Hiding Fingers...");
            }
         }
         if (ImGui.menuItem(labels.get("Hide All Frames"), "", hideAll))
         {
            if (hideAll.get())
            {
               RDXBaseUI.pushNotification("Hiding All Frames...");
            }
         }
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("Be ready to connect to Captury Live before turning on.");
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

   public void addToActorArray(int actorID)
   {
      actorArray.add(actorID);
   }

   public int getLastActorArray()
   {
      return actorArray.get(actorArray.size() - 1);
   }

   public int getSelectedActorID()
   {
      return selectedActorID;
   }

   public void setIpAddress(String ipAddress)
   {
      this.ipAddress = ipAddress;
   }

   public void setPort(int port)
   {
      this.port = port;
   }

   public boolean checkContain(int actorID)
   {
      return actorArray.contains(actorID);
   }
}
