package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;

import java.util.ArrayList;

public class RDX3DPanelToolbar
{
   private boolean show = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final float iconSize = 40.0f;
   private final ArrayList<RDX3DPanelToolbarButton> buttons = new ArrayList<>();

   public RDX3DPanelToolbarButton addButton()
   {
      RDX3DPanelToolbarButton toolbarButton = new RDX3DPanelToolbarButton();
      buttons.add(toolbarButton);
      return toolbarButton;
   }

   public void render(float mainWindowWidth, float mainWindowPosX, float mainWindowPosY)
   {
      if (buttons.size() > 0 && show)
      {
         float panelWidth = iconSize * buttons.size() + 9.0f * buttons.size() - 1.0f;
         float panelHeight = iconSize + 16.0f;

         ImGui.setNextWindowSize(panelWidth, panelHeight);
         float centerX = mainWindowPosX + mainWindowWidth / 2;
         float startX = centerX - panelWidth / 2;
         ImGui.setNextWindowPos(startX, mainWindowPosY + 15.0f);

         int windowFlags = ImGuiWindowFlags.NoTitleBar; // undecorated
         ImGui.begin(labels.get("Toolbar"), windowFlags);

         for (RDX3DPanelToolbarButton button : buttons)
         {
            RDXIconTexture icon = button.getAppropriateIcon();
            if (icon == null)
               continue;

            float sizeX = iconSize;
            float sizeY = iconSize;
            float uv0X = 0.0f;
            float uv0Y = 0.0f;
            float uv1X = 1.0f / 3.0f;
            float uv1Y = 1.0f;
            if (button.getHovered())
            {
               uv0X = 1.0f / 3.0f;
               uv1X = 2.0f / 3.0f;
            }
            else if (button.getDown())
            {
               uv0X = 2.0f / 3.0f;
               uv1X = 1.0f;
            }
            float bgColorR = 1.0f;
            float bgColorG = 1.0f;
            float bgColorB = 1.0f;
            float bgColorA = 0.0f;
            // The "tint" is another rectangle drawn over the button
            float tintR = 1.0f;
            float tintG = 1.0f;
            float tintB = 1.0f;
            float tintA = 1.0f;
            int framePadding = 0;
            if (ImGui.imageButton(icon.getTexture().getTextureObjectHandle(), sizeX, sizeY, uv0X, uv0Y, uv1X, uv1Y, framePadding,
                                  bgColorR, bgColorG, bgColorB, bgColorA, tintR, tintG, tintB, tintA))
            {
               button.onPressed();
            }
            button.setHovered(ImGui.isItemHovered());
            button.setDown(button.getHovered() && ImGui.isMouseDown(ImGuiMouseButton.Left));

            if (button.getTooltipText() != null)
            {
               ImGuiTools.previousWidgetTooltip(button.getTooltipText());
            }
            ImGui.sameLine();
         }
         ImGui.end();
      }
   }

   public boolean isShow()
   {
      return show;
   }

   public void setShow(boolean show)
   {
      this.show = show;
   }
}
