package us.ihmc.gdx.imgui;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class ImGuiGlfwWindow
{
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final GDXImGuiWindowAndDockSystem imGuiDockSystem = new GDXImGuiWindowAndDockSystem();
   private final ImGuiDockingSetup dockingSetup = new ImGuiDockingSetup();

   public ImGuiGlfwWindow(String windowTitle, int windowWidth, int windowHeight)
   {
      glfwWindowForImGui = new GlfwWindowForImGui(windowTitle, windowWidth, windowHeight);
   }

   public void run(Runnable render, Runnable dispose)
   {
      glfwWindowForImGui.create();

      long windowHandle = glfwWindowForImGui.getWindowHandle();

      imGuiDockSystem.create(windowHandle);

      while (!glfwWindowShouldClose(windowHandle))
      {
         ImGuiTools.glClearDarkGray();
         imGuiDockSystem.beforeWindowManagement();

         render.run();

         if (imGuiDockSystem.isFirstRenderCall())
         {
            dockingSetup.build(imGuiDockSystem.getCentralDockspaceId());
         }

         imGuiDockSystem.afterWindowManagement();
      }

      dispose.run();

      imGuiDockSystem.dispose();

      glfwWindowForImGui.dispose();
   }

   public ImGuiDockingSetup getDockingSetup()
   {
      return dockingSetup;
   }
}
