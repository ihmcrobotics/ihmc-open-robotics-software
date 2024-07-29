package us.ihmc.rdx.Captury;

import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXCapturyManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean capturyEnabled = new ImBoolean(false);
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
   }

   public ImBoolean getCapturyEnabled()
   {
      return capturyEnabled;
   }
}
