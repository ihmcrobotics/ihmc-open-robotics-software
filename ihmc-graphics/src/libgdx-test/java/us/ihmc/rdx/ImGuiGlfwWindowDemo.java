package us.ihmc.rdx;

import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.*;

public class ImGuiGlfwWindowDemo
{
   public ImGuiGlfwWindowDemo()
   {
      ImGuiGlfwWindow imGuiGlfwWindow = new ImGuiGlfwWindow(getClass());
      imGuiGlfwWindow.getPanelManager().addSelfManagedPanel("Window");
      imGuiGlfwWindow.run(this::create, this::render, this::dispose);
   }

   public void create()
   {

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
