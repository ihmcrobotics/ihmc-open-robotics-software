package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.ui.GDXImGuiPerspectiveManager;
import us.ihmc.gdx.ui.ImGuiConfigurationLocation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;

import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;

public class ImGuiGlfwWindow
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridFile windowSettingsFile;
   private final Stopwatch runTime = new Stopwatch().start();
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final GDXImGuiPerspectiveManager perspectiveManager;

   public ImGuiGlfwWindow(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder, classForLoading.getSimpleName());
   }

   public ImGuiGlfwWindow(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String windowTitle)
   {
      configurationExtraPath = "/configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridDirectory(dotIHMCDirectory,
                                                       directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForLoading,
                                                       configurationExtraPath);

      imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem();
      glfwWindowForImGui = new GlfwWindowForImGui(windowTitle);
      perspectiveManager = new GDXImGuiPerspectiveManager(classForLoading,
                                                          directoryNameToAssumePresent,
                                                          subsequentPathToResourceFolder,
                                                          configurationExtraPath,
                                                          configurationBaseDirectory,
      updatedPerspectiveDirectory ->
      {
         windowSettingsFile = new HybridFile(updatedPerspectiveDirectory, "WindowSettings.json");
         imGuiWindowAndDockSystem.setDirectory(updatedPerspectiveDirectory);
      },
      loadConfigurationLocation ->
      {
         imGuiWindowAndDockSystem.loadConfiguration(loadConfigurationLocation);
         Path libGDXFile = loadConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL
               ? windowSettingsFile.getWorkspaceFile() : windowSettingsFile.getExternalFile();
         JSONFileTools.load(libGDXFile, jsonNode ->
         {
            int width = jsonNode.get("windowWidth").asInt();
            int height = jsonNode.get("windowHeight").asInt();
            glfwWindowForImGui.setWindowSize(width, height);
         });
      },
      saveConfigurationLocation ->
      {
         saveApplicationSettings(saveConfigurationLocation);
      });
   }

   public void run(Runnable render, Runnable dispose)
   {
      run(null, render, dispose);
   }

   public void run(Runnable configure, Runnable render, Runnable dispose)
   {
      JSONFileTools.loadUserWithClasspathDefaultFallback(windowSettingsFile, jsonNode ->
      {
         glfwWindowForImGui.setWindowSize(jsonNode.get("windowWidth").asInt(), jsonNode.get("windowHeight").asInt());
      });

      glfwWindowForImGui.create();

      long windowHandle = glfwWindowForImGui.getWindowHandle();

      imGuiWindowAndDockSystem.create(windowHandle);

      while (!glfwWindowShouldClose(windowHandle))
      {
         ImGuiTools.glClearDarkGray();

         if (configure != null)
         {
            configure.run();
         }

         imGuiWindowAndDockSystem.beforeWindowManagement();
         renderMenuBar();

         render.run();

         imGuiWindowAndDockSystem.afterWindowManagement();
      }

      dispose.run();

      imGuiWindowAndDockSystem.dispose();

      glfwWindowForImGui.dispose();
   }

   private void renderMenuBar()
   {
      ImGui.beginMainMenuBar();
      perspectiveManager.renderImGuiPerspectiveMenu();
      if (ImGui.beginMenu("Panels"))
      {
         imGuiWindowAndDockSystem.renderMenuDockPanelItems();
         ImGui.endMenu();
      }
      ImGui.sameLine(ImGui.getWindowSizeX() - 70.0f);
      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");
      ImGui.endMainMenuBar();
   }

   private void saveApplicationSettings(ImGuiConfigurationLocation saveConfigurationLocation)
   {
      imGuiWindowAndDockSystem.saveConfiguration(saveConfigurationLocation);
      Consumer<ObjectNode> rootConsumer = root ->
      {
         root.put("windowWidth", glfwWindowForImGui.getWindowWidth());
         root.put("windowHeight", glfwWindowForImGui.getWindowHeight());
      };
      if (saveConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL)
      {
         LogTools.info("Saving window settings to {}", windowSettingsFile.getWorkspaceFile().toString());
         JSONFileTools.save(windowSettingsFile.getWorkspaceFile(), rootConsumer);
      }
      else
      {
         LogTools.info("Saving window settings to {}", windowSettingsFile.getExternalFile().toString());
         JSONFileTools.save(windowSettingsFile.getExternalFile(), rootConsumer);
      }
   }

   public void runWithSinglePanel(Runnable renderImGuiWidgets)
   {
      ImGuiPanel mainPanel = new ImGuiPanel("Main Panel", renderImGuiWidgets);
      mainPanel.getIsShowing().set(true);
      imGuiWindowAndDockSystem.getPanelManager().addPanel(mainPanel);
      ThreadTools.startAThread(() ->
      {
         run(() -> { }, () -> System.exit(0));
      }, glfwWindowForImGui.getWindowTitle());
   }

   public ImGuiPanelManager getPanelManager()
   {
      return imGuiWindowAndDockSystem.getPanelManager();
   }

   public GDXImGuiWindowAndDockSystem getImGuiDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }
}
