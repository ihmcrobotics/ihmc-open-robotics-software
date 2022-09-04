package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

// TODO: Extract tool bar method from GDX3DPanel
public class GDX3DPanelToolbar
{
   private final float iconSize = 35.0f;
   private final float gap = 17.7f;
   private ArrayList<GDX3DPanelToolbarButton> buttons = new ArrayList<>();
   private final ImBoolean showFlag = new ImBoolean(true);

   // with tab bar
   //            int windowFlags = ImGuiWindowFlags.None;
   //            panelHeight += ImGuiTools.TAB_BAR_HEIGHT;

   // FIXME: Need proper (generalized) way to implement toggling later.
   private int stateIndex = 0;

   // TODO: Implement methods here
   public void render(float mainWindowWidth, float mainWindowPosX, float mainWindowPosY)
   {
      // NOTE: Make Hot key button panel here.
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
         // no tab bar
         int windowFlags = ImGuiWindowFlags.NoTitleBar;
         ImGui.begin("Testing . . .", showFlag, windowFlags);

         for (int i = 0; i < buttons.size(); ++i)
         {
            GDX3DPanelToolbarButton button = buttons.get(i);

            // button clicked.
            if (ImGui.imageButton(button.getIcon().getTexture().getTextureObjectHandle(), iconSize, iconSize))
            {
               button.isClicked();
               if (button.isTogglable())
               {
                  stateIndex = button.getStateIndex();
               }
               if (button.doesDepend())
               {
                  button.setState(stateIndex);
               }
               button.execute();
            }

            if (!button.getToolTipText().isEmpty())
            {
               ImGuiTools.previousWidgetTooltip(button.getToolTipText());
            }
            ImGui.sameLine();
         }
         ImGui.end();
      }
   }

   public void addButton(GDX3DPanelToolbarButton gdx3DPanelToolbarButton)
   {
      buttons.add(gdx3DPanelToolbarButton);
   }
}
