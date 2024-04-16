package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
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
//      if (buttons.size() > 0 && show)
//      {
//         float spaceBetweenButtons = 9.0f;
//         float toolbarWidth = buttons.size() * (iconSize + spaceBetweenButtons);
//
//         float centerX = mainWindowWidth / 2.0f;
//         float startX = centerX - toolbarWidth / 2.0f;
//         float startY = 40.0f;
//
//         float upGray = 1.0f;
//         float hoverGray = 0.4f;
//         float downGray = 0.2f;
//         ImGui.pushStyleColor(ImGuiCol.Button, upGray, upGray, upGray, 0.5f);
//         ImGui.pushStyleColor(ImGuiCol.ButtonHovered, hoverGray, hoverGray, hoverGray, 0.8f);
//         ImGui.pushStyleColor(ImGuiCol.ButtonActive, downGray, downGray, downGray, 0.9f);
//
//         for (int i = 0; i < buttons.size(); i++)
//         {
//            RDX3DPanelToolbarButton button = buttons.get(i);
//            RDXIconTexture icon = button.getIconTexture();
//            if (icon == null)
//               continue;
//
//            ImGui.setCursorPos(startX + (i * (iconSize + spaceBetweenButtons)), startY);
//
//            float sizeX = iconSize;
//            float sizeY = iconSize;
//            float uv0X = 0.0f;
//            float uv0Y = 0.0f;
//            float uv1X = 1.0f;
//            float uv1Y = 1.0f;
//            // We are making this background fully transparent currently
//            float backgroundColorR = 1.0f;
//            float backgroundColorG = 1.0f;
//            float backgroundColorB = 1.0f;
//            float backgroundColorA = 0.0f;
//            // The "tint" is a fade of the full button look. White and 1.0f alpha
//            // is the only value that does not "tamper" with the button look.
//            float tintR = 1.0f;
//            float tintG = 1.0f;
//            float tintB = 1.0f;
//            float tintA = 1.0f;
//            int framePadding = 0;
//            // An ImGui imageButton must have a consistent texture object handle or it will not operate correctly.
//            // You must use UV coordinates to change the graphic based on state. (You could also reupload the texture
//            // to the GPU, but that's probably not the best method.)
//            if (ImGui.imageButton(icon.getTexture().getTextureObjectHandle(),
//                                  sizeX,
//                                  sizeY,
//                                  uv0X,
//                                  uv0Y,
//                                  uv1X,
//                                  uv1Y,
//                                  framePadding,
//                                  backgroundColorR,
//                                  backgroundColorG,
//                                  backgroundColorB,
//                                  backgroundColorA,
//                                  tintR,
//                                  tintG,
//                                  tintB,
//                                  tintA))
//            {
//               button.onPressed();
//            }
//            button.setHovered(ImGui.isItemHovered());
//            button.setDown(button.getHovered() && ImGui.isMouseDown(ImGuiMouseButton.Left));
//
//            if (!ImGui.isItemHovered())
//            {
//               button.getTooltipTimer().reset();
//            }
//
//            if (button.getTooltipText() != null && button.getTooltipTimer().isExpired(0.7))
//            {
//               ImGui.setTooltip(button.getTooltipText());
//            }
//         }
//
//         ImGui.popStyleColor(3);
//      }
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
