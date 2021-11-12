package us.ihmc.gdx;

import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.*;

public class ImGuiGlfwWindowDemo
{
   public ImGuiGlfwWindowDemo()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass(), "ihmc-open-robotics-software", "ihmc-graphics/src/libgdx-test/resources");
      imGuiGlfwWindow.getPanelManager().addPrimaryPanel("Window");
      imGuiGlfwWindow.run(this::render, this::dispose);
   }

   public void render()
   {
      ImGui.begin("Window");
      ImGui.text("Text");
      ImGui.end();

      ImGui.showDemoWindow();
   }

   public void dispose()
   {

   }

   public static void main(String[] args)
   {
      new ImGuiGlfwWindowDemo();
   }
}
