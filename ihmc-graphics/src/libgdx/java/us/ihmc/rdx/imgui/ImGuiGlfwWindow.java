package us.ihmc.rdx.imgui;

import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.ui.GDXImGuiPerspectiveManager;
import us.ihmc.rdx.ui.ImGuiConfigurationLocation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;

public class ImGuiGlfwWindow
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridFile windowSettingsFile;
   private final FrequencyCalculator fpsCalculator = new FrequencyCalculator();
   private final Stopwatch runTime = new Stopwatch().start();
   private String[] iconPaths = null;
   private final GlfwWindowForImGui glfwWindowForImGui;
   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final GDXImGuiPerspectiveManager perspectiveManager;
   private final ImBoolean vsync = new ImBoolean(true);
   private final ImInt maxFrameRate = new ImInt(240);

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
                                                          configurationBaseDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(updatedPerspectiveDirectory ->
      {
         windowSettingsFile = new HybridFile(updatedPerspectiveDirectory, "WindowSettings.json");
      });
      perspectiveManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      perspectiveManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         windowSettingsFile.setMode(loadConfigurationLocation.toHybridResourceMode());
         JSONFileTools.load(windowSettingsFile.getInputStream(), jsonNode ->
         {
            int width = jsonNode.get("windowWidth").asInt();
            int height = jsonNode.get("windowHeight").asInt();
            glfwWindowForImGui.setWindowSize(width, height);
         });
      });
      perspectiveManager.getSaveListeners().add(this::saveApplicationSettings);
      perspectiveManager.applyPerspectiveDirectory();
   }

   public void run(Runnable create, Runnable render, Runnable dispose)
   {
      run(create, null, render, dispose);
   }

   public void run(Runnable create, Runnable configure, Runnable render, Runnable dispose)
   {
      JSONFileTools.loadUserWithClasspathDefaultFallback(windowSettingsFile, jsonNode ->
      {
         glfwWindowForImGui.setWindowSize(jsonNode.get("windowWidth").asInt(), jsonNode.get("windowHeight").asInt());
      });

      glfwWindowForImGui.create();
      if (iconPaths != null)
         glfwWindowForImGui.setIcon(iconPaths);

      long windowHandle = glfwWindowForImGui.getWindowHandle();

      imGuiWindowAndDockSystem.create(windowHandle);

      if (create != null)
         create.run();

      glfwWindowForImGui.launch(() ->
      {
         if (configure != null)
         {
            configure.run();
         }

         imGuiWindowAndDockSystem.beforeWindowManagement();
         renderMenuBar();

         render.run();

         imGuiWindowAndDockSystem.afterWindowManagement();
      });

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
      if (ImGui.beginMenu("Settings"))
      {
         ImGui.pushItemWidth(80.0f);
         if (ImGui.checkbox("Vsync", vsync))
         {
            glfwWindowForImGui.setVSyncEnabled(vsync.get());
         }
         if (ImGui.inputInt("Max Frame Rate", maxFrameRate, 1))
         {
            glfwWindowForImGui.setMaxFrameRate(maxFrameRate.get());
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
      ImGui.sameLine(ImGui.getWindowSizeX() - 110.0f);
      fpsCalculator.ping();
      String fpsString = String.valueOf((int) fpsCalculator.getFrequency());
      while (fpsString.length() < 3)
      {
         fpsString = " " + fpsString;
      }
      ImGui.text(fpsString + " Hz");
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
      ThreadTools.startAThread(() -> run(null, () -> { }, () -> System.exit(0)), glfwWindowForImGui.getWindowTitle());
   }

   public void setIcons(String... iconPaths)
   {
      this.iconPaths = iconPaths;
   }

   public void setEnableVsync(boolean enableVsync)
   {
      vsync.set(enableVsync);
      glfwWindowForImGui.setVSyncEnabled(enableVsync);
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
