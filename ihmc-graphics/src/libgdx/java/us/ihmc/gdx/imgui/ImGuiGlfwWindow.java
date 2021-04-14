package us.ihmc.gdx.imgui;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class ImGuiGlfwWindow
{
   private final String windowTitle;
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final GDXImGuiWindowAndDockSystem imGuiDockSystem = new GDXImGuiWindowAndDockSystem();
   private final ImGuiDockingSetup dockingSetup = new ImGuiDockingSetup();

   public ImGuiGlfwWindow(String windowTitle, int windowWidth, int windowHeight)
   {
      this.windowTitle = windowTitle;
      glfwWindowForImGui = new GlfwWindowForImGui(windowTitle, windowWidth, windowHeight);
   }

   public void run(Runnable render, Runnable dispose)
   {
      run(null, render, dispose);
   }

   public void run(Runnable configure, Runnable render, Runnable dispose)
   {
      glfwWindowForImGui.create();

      long windowHandle = glfwWindowForImGui.getWindowHandle();

      imGuiDockSystem.create(windowHandle);

      while (!glfwWindowShouldClose(windowHandle))
      {
         ImGuiTools.glClearDarkGray();

         if (configure != null)
         {
            configure.run();
         }

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

   public GDXImGuiWindowAndDockSystem getImGuiDockSystem()
   {
      return imGuiDockSystem;
   }
}
