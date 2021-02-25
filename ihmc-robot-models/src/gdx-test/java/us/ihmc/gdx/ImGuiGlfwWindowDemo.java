package us.ihmc.gdx;

import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.GDXImGuiWindowAndDockSystem;
import us.ihmc.gdx.imgui.GlfwWindowForImGui;
import us.ihmc.gdx.imgui.ImGuiDockingSetup;
import us.ihmc.gdx.imgui.ImGuiTools;

import static org.lwjgl.glfw.GLFW.*;

public class ImGuiGlfwWindowDemo
{
   private final GlfwWindowForImGui glfwWindowForImGui = new GlfwWindowForImGui();
   private final GDXImGuiWindowAndDockSystem imGuiDockSystem = new GDXImGuiWindowAndDockSystem();

   public ImGuiGlfwWindowDemo()
   {
      glfwWindowForImGui.create();

      long windowHandle = glfwWindowForImGui.getWindowHandle();

      imGuiDockSystem.create(windowHandle);

      ImGuiDockingSetup dockingSetup = new ImGuiDockingSetup();
      dockingSetup.addFirst("Meow");

      while (!glfwWindowShouldClose(windowHandle))
      {
         ImGuiTools.glClearDarkGray();
         imGuiDockSystem.beforeWindowManagement();

         ImGui.begin("Meow");
         ImGui.text("Hello");
         ImGui.end();

         if (imGuiDockSystem.isFirstRenderCall())
         {
            dockingSetup.build(imGuiDockSystem.getCentralDockspaceId());
         }

         imGuiDockSystem.afterWindowManagement();
      }

      imGuiDockSystem.dispose();

      glfwWindowForImGui.dispose();
   }

   public static void main(String[] args)
   {
      new ImGuiGlfwWindowDemo();
   }
}
