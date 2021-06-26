package us.ihmc.gdx.imgui;

import us.ihmc.tools.io.HybridDirectory;

import java.nio.file.Paths;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class ImGuiGlfwWindow
{
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final Class<? extends ImGuiGlfwWindow> classForLoading = getClass();
   private final String directoryNameToAssumePresent = "ihmc-open-robotics-software";
   private final String subsequentPathToResourceFolder = "ihmc-graphics/src/libgdx/resources";
   private final HybridDirectory configurationDirectory = new HybridDirectory(Paths.get(System.getProperty("user.home"), ".ihmc"),
                                                                              directoryNameToAssumePresent,
                                                                              subsequentPathToResourceFolder,
                                                                              classForLoading,
                                                                              "GLFWDemo");
   private final GDXImGuiWindowAndDockSystem imGuiDockSystem = new GDXImGuiWindowAndDockSystem(configurationDirectory);
   private final ImGuiPanelManager panelManager = new ImGuiPanelManager(configurationDirectory);
   private boolean isFirstRenderCall = true;

   public ImGuiGlfwWindow(String windowTitle, int windowWidth, int windowHeight)
   {
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

         imGuiDockSystem.beforeWindowManagement(isFirstRenderCall);

         render.run();

         imGuiDockSystem.afterWindowManagement();
         isFirstRenderCall = false;
      }

      dispose.run();

      imGuiDockSystem.dispose();

      glfwWindowForImGui.dispose();
   }

   public ImGuiPanelManager getPanelManager()
   {
      return panelManager;
   }

   public GDXImGuiWindowAndDockSystem getImGuiDockSystem()
   {
      return imGuiDockSystem;
   }
}
