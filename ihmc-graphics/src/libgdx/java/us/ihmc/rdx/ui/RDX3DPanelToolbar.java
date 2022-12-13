package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;

import java.util.ArrayList;

public class RDX3DPanelToolbar
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final float iconSize = 35.0f;
   private final float gap = 17.7f;
   private final ArrayList<RDX3DPanelToolbarButton> buttons = new ArrayList<>();

   public RDX3DPanelToolbarButton addButton()
   {
      RDX3DPanelToolbarButton toolbarButton = new RDX3DPanelToolbarButton();
      buttons.add(toolbarButton);
      return toolbarButton;
   }

   public void render(float mainWindowWidth, float mainWindowPosX, float mainWindowPosY)
   {
      if (buttons.size() > 0)
      {
         int numButtons = buttons.size();
         float offsetY = 12.0f;
         float panelWidth = iconSize * numButtons + gap * numButtons;
         float panelHeight = iconSize + 2 * offsetY;

         ImGui.setNextWindowSize(panelWidth, panelHeight);
         float centerX = mainWindowPosX + mainWindowWidth / 2;
         float startX = centerX - panelWidth / 2;
         ImGui.setNextWindowPos(startX, mainWindowPosY + 15.0f);

         int windowFlags = ImGuiWindowFlags.NoTitleBar; // undecorated
         ImGui.begin(labels.get("Toolbar"), windowFlags);

         for (RDX3DPanelToolbarButton button : buttons)
         {
            RDXIconTexture icon = button.getIcon();
            if (icon == null)
               continue;

            if (ImGui.imageButton(icon.getTexture().getTextureObjectHandle(), iconSize, iconSize))
            {
               button.onPressed();
            }

            if (button.getTooltipText() != null)
            {
               ImGuiTools.previousWidgetTooltip(button.getTooltipText());
            }
            ImGui.sameLine();
         }
         ImGui.end();
      }
   }
}
