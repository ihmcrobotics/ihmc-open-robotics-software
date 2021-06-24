package us.ihmc.gdx.imgui;

import java.nio.file.Path;
import java.nio.file.Paths;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class ImGuiGlfwWindow
{
   private final String windowTitle;
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final Class<? extends ImGuiGlfwWindow> classForLoading = getClass();
   private final String directoryNameToAssumePresent = "ihmc-open-robotics-software";
   private final String subsequentPathToResourceFolder = "ihmc-graphics/src/libgdx/resources";
   private final Path imGuiUserSettingsPath = Paths.get(System.getProperty("user.home"), ".ihmc/" + "GLFWDemo" + "ImGuiSettings.ini")
                                                   .toAbsolutePath()
                                                   .normalize();
   private final GDXImGuiWindowAndDockSystem imGuiDockSystem = new GDXImGuiWindowAndDockSystem(classForLoading,
                                                                                               directoryNameToAssumePresent,
                                                                                               subsequentPathToResourceFolder,
                                                                                               imGuiUserSettingsPath);
   private final ImGuiPanelManager panelManager = new ImGuiPanelManager(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder);

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

         imGuiDockSystem.afterWindowManagement();
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
