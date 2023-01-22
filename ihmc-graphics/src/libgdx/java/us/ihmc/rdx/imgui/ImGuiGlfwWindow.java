package us.ihmc.rdx.imgui;

import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.ui.RDXImGuiLayoutManager;
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
   private final RDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final RDXImGuiLayoutManager layoutManager;
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

      imGuiWindowAndDockSystem = new RDXImGuiWindowAndDockSystem();
      glfwWindowForImGui = new GlfwWindowForImGui(windowTitle);
      layoutManager = new RDXImGuiLayoutManager(classForLoading,
                                                directoryNameToAssumePresent,
                                                subsequentPathToResourceFolder,
                                                configurationExtraPath,
                                                configurationBaseDirectory);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(updatedLayoutDirectory ->
      {
         windowSettingsFile = new HybridFile(updatedLayoutDirectory, "WindowSettings.json");
      });
      layoutManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      layoutManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         windowSettingsFile.setMode(loadConfigurationLocation.toHybridResourceMode());
         InputStream inputStream = windowSettingsFile.getInputStream();
         if (inputStream != null)
         {
            JSONFileTools.load(inputStream, jsonNode ->
            {
               int width = jsonNode.get("windowWidth").asInt();
               int height = jsonNode.get("windowHeight").asInt();
               glfwWindowForImGui.setWindowSize(width, height);
            });
         }
         return inputStream != null;
      });
      layoutManager.getSaveListeners().add(this::saveApplicationSettings);
      layoutManager.applyLayoutDirectory();
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
      layoutManager.renderImGuiLayoutMenu();
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
      if (saveConfigurationLocation.isVersionControl())
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

   public RDXImGuiWindowAndDockSystem getImGuiDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }
}
