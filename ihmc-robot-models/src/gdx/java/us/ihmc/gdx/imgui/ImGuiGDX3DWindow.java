package us.ihmc.gdx.imgui;

import imgui.ImColor;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import us.ihmc.gdx.GDX3DApplication;

import static us.ihmc.gdx.GDX3DApplication.CLEAR_COLOR;

public class ImGuiGDX3DWindow
{
   private static final String WINDOW_NAME = "3D View";

   public void renderBeforeOtherWindows(GDX3DApplication gdx3DApplication)
   {
      // TODO: Pass inputs through ImGui?
      int flags = ImGuiWindowFlags.None;
      //         flags += ImGuiWindowFlags.NoNavInputs;
               flags += ImGuiWindowFlags.NoTitleBar;
      ImGui.begin(WINDOW_NAME, flags);

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY();
      float sizeX = ImGui.getWindowSizeX();
      float sizeY = ImGui.getWindowSizeY();
      gdx3DApplication.setViewportBounds((int) posX, gdx3DApplication.getCurrentWindowHeight() - (int) posY - (int) sizeY, (int) sizeX, (int) sizeY);

      ImGui.getWindowDrawList().addRectFilled(posX, posY, posX + sizeX, posY + sizeY, ImColor.floatToColor(CLEAR_COLOR, CLEAR_COLOR, CLEAR_COLOR, 1.0f));

      ImGui.end();

      gdx3DApplication.getCamera3D().clearInputExclusionBoxes();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
