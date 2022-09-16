package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

import java.util.ArrayList;

public class GDX3DPanelToolbar
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final float iconSize = 35.0f;
   private final float gap = 17.7f;
   private final ArrayList<GDX3DPanelToolbarButton> buttons = new ArrayList<>();

   public GDX3DPanelToolbarButton addButton()
   {
      GDX3DPanelToolbarButton toolbarButton = new GDX3DPanelToolbarButton();
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

         for (GDX3DPanelToolbarButton button : buttons)
         {
            if (ImGui.imageButton(button.getIcon().getTexture().getTextureObjectHandle(), iconSize, iconSize))
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
