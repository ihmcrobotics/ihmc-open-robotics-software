package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.tools.GDXToolButton;
import java.util.ArrayList;

// TODO: Extract tool bar method from GDX3DPanel
public class GDXToolBar
{
   private final float iconSize = 35.0f;
   private final float gap = 17.7f;
   private ArrayList<GDXToolButton> buttons = new ArrayList<>();
   private final ImBoolean showFlag = new ImBoolean(true);

   // with tab bar
   //            int windowFlags = ImGuiWindowFlags.None;
   //            panelHei += ImGuiTools.TAB_BAR_HEIGHT;

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
         float panelHei = iconSize + 2 * offsetY;

         ImGui.setNextWindowSize(panelWidth, panelHei);
         float centerX = mainWindowPosX + mainWindowWidth / 2;
         float startX = centerX - panelWidth / 2;
         ImGui.setNextWindowPos(startX, mainWindowPosY + 15.0f);
         // no tab bar
         int windowFlags = ImGuiWindowFlags.NoTitleBar;
         ImGui.begin("Testing . . .", showFlag, windowFlags);

         for (int i = 0; i < buttons.size(); ++i)
         {
            GDXToolButton button = buttons.get(i);

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

   public void addButton(GDXToolButton gdxToolButton)
   {
      buttons.add(gdxToolButton);
   }
}
