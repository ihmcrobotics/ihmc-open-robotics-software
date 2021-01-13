package us.ihmc.gdx.imgui;

import imgui.ImColor;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;

import static us.ihmc.gdx.sceneManager.GDX3DSceneManager.CLEAR_COLOR;

public class ImGuiGDX3DWindow
{
   private static final String WINDOW_NAME = "3D View";

   public void renderBeforeOtherWindows(GDX3DSceneManager sceneManager)
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
      sceneManager.setViewportBounds((int) posX, sceneManager.getCurrentWindowHeight() - (int) posY - (int) sizeY, (int) sizeX, (int) sizeY);

      ImGui.getWindowDrawList().addRectFilled(posX, posY, posX + sizeX, posY + sizeY, ImColor.floatToColor(CLEAR_COLOR, CLEAR_COLOR, CLEAR_COLOR, 1.0f));

      ImGui.end();

      sceneManager.getCamera3D().clearInputExclusionBoxes();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
