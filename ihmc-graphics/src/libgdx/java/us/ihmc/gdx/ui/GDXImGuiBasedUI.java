package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class GDXImGuiBasedUI
{
   public static final int ANTI_ALIASING = 2;

   private static boolean RECORD_VIDEO = Boolean.parseBoolean(System.getProperty("record.video"));
   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active
   private static final String VIEW_3D_WINDOW_NAME = "3D View";

   private GLProfiler glProfiler;
   private final GDX3DScene primaryScene = new GDX3DScene();
   private final GDX3DPanel primary3DPanel;
   private final ArrayList<GDX3DPanel> additional3DPanels = new ArrayList<>();
   private final GDXVRManager vrManager = new GDXVRManager();
   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
//   private final GDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridFile libGDXSettingsFile;
   private final FrequencyCalculator fpsCalculator = new FrequencyCalculator();
   private final Stopwatch runTime = new Stopwatch().start();
   private String statusText = ""; // TODO: Add status at bottom of window
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt foregroundFPS = new ImInt(240);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImBoolean shadows = new ImBoolean(false);
   private final ImBoolean middleClickOrbit = new ImBoolean(false);
   private final ImFloat backgroundShade = new ImFloat(GDX3DSceneTools.CLEAR_COLOR);
   private final ImInt libGDXLogLevel = new ImInt(GDXTools.toGDX(LogTools.getLevel()));
   private final ImFloat imguiFontScale = new ImFloat(1.0f);
   private final GDXImGuiPerspectiveManager perspectiveManager;
   private long renderIndex = 0;
   private double isoZoomOut = 0.7;

   public GDXImGuiBasedUI(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder, classForLoading.getSimpleName());
   }

   public GDXImGuiBasedUI(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String windowTitle)
   {
      this.windowTitle = windowTitle;

      configurationExtraPath = "/configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridDirectory(dotIHMCDirectory,
                                                       directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForLoading,
                                                       configurationExtraPath);

      imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem();
      perspectiveManager = new GDXImGuiPerspectiveManager(classForLoading,
                                                          directoryNameToAssumePresent,
                                                          subsequentPathToResourceFolder,
                                                          configurationExtraPath,
                                                          configurationBaseDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(updatedPerspectiveDirectory ->
      {
         libGDXSettingsFile = new HybridFile(updatedPerspectiveDirectory, "GDXSettings.json");
      });
      perspectiveManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      perspectiveManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         libGDXSettingsFile.setMode(loadConfigurationLocation.toHybridResourceMode());
         LogTools.info("Loading libGDX settings from {}", libGDXSettingsFile.getLocationOfResourceForReading());
         JSONFileTools.load(libGDXSettingsFile.getInputStream(), jsonNode ->
         {
            int width = jsonNode.get("windowWidth").asInt();
            int height = jsonNode.get("windowHeight").asInt();
            Gdx.graphics.setWindowedMode(width, height);
         });
      });
      perspectiveManager.getSaveListeners().add(this::saveApplicationSettings);
      perspectiveManager.applyPerspectiveDirectory();

//      guiRecorder = new GDXLinuxGUIRecorder(24, 0.8f, getClass().getSimpleName());
//      onCloseRequestListeners.add(guiRecorder::stop);
//      Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

      if (RECORD_VIDEO)
      {
         //         ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
//         ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
      }

      primary3DPanel = new GDX3DPanel(VIEW_3D_WINDOW_NAME, ANTI_ALIASING, true);
   }

   public void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter)
   {
      AtomicReference<Integer> windowWidth = new AtomicReference<>(800);
      AtomicReference<Integer> windowHeight = new AtomicReference<>(600);
      JSONFileTools.loadUserWithClasspathDefaultFallback(libGDXSettingsFile, jsonNode ->
      {
         windowWidth.set(jsonNode.get("windowWidth").asInt());
         windowHeight.set(jsonNode.get("windowHeight").asInt());
      });

      LogTools.info("Launching GDX application");
      GDXApplicationCreator.launchGDXApplication(applicationAdapter, windowTitle, windowWidth.get(), windowHeight.get());
   }

   public void create()
   {
      create(GDXSceneLevel.MODEL, GDXSceneLevel.VIRTUAL);
   }

   public void create(GDXSceneLevel... sceneLevels)
   {
      LogTools.info("Creating...");
      GDXTools.printGLVersion();

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = GDXTools.createGLProfiler();

      primaryScene.create(sceneLevels);
      primaryScene.addDefaultLighting();
      primary3DPanel.create(GDXInputMode.ImGui, glProfiler, primaryScene);
      imGuiWindowAndDockSystem.getPanelManager().addPanel(primary3DPanel.getImGuiPanel());
      primary3DPanel.getImGuiPanel().getIsShowing().set(true);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?

      primary3DPanel.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      primaryScene.addCoordinateFrame(0.3);

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));

      vrManager.create();
      primaryScene.addRenderableProvider(vrManager::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
      primary3DPanel.addImGui3DViewPickCalculator(vrManager::calculate3DViewPick);
      primary3DPanel.addImGui3DViewInputProcessor(vrManager::process3DViewInput);
      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Thread Debugger", vrManager::renderImGuiDebugWidgets);
      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Settings", vrManager::renderImGuiTunerWidgets);
   }

   public void renderBeforeOnScreenUI()
   {
      vrManager.pollEventsAndRender(this, primaryScene);
      Gdx.graphics.setTitle(windowTitle);
      imGuiWindowAndDockSystem.beforeWindowManagement();
      primary3DPanel.render();
      for (GDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.render();
      }
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();
      ++renderIndex;
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
         if (ImGuiTools.volatileInputInt(labels.get("Foreground FPS Limit"), foregroundFPS, 1))
         {
            Gdx.graphics.setForegroundFPS(foregroundFPS.get());
         }
         if (ImGui.checkbox(labels.get("Vsync"), vsync))
         {
            Gdx.graphics.setVSync(vsync.get());
         }
         if (ImGui.checkbox(labels.get("Shadows"), shadows))
         {
            primaryScene.setShadowsEnabled(shadows.get());
         }
         if (ImGuiTools.volatileInputInt(labels.get("libGDX log level"), libGDXLogLevel, 1))
         {
            Gdx.app.setLogLevel(libGDXLogLevel.get());
         }
         if (ImGuiTools.volatileInputFloat(labels.get("Font Size"), imguiFontScale, 0.1f))
         {
            ImGui.getIO().setFontGlobalScale(imguiFontScale.get());
         }
         ImGui.separator();
         boolean renderingGroundTruthEnvironment = primaryScene.getSceneLevelsToRender().contains(GDXSceneLevel.GROUND_TRUTH);
         if (ImGui.checkbox(labels.get("Render Ground Truth Environment"), renderingGroundTruthEnvironment))
         {
            if (renderingGroundTruthEnvironment)
               primaryScene.getSceneLevelsToRender().remove(GDXSceneLevel.GROUND_TRUTH);
            else
               primaryScene.getSceneLevelsToRender().add(GDXSceneLevel.GROUND_TRUTH);
         }
         boolean renderingModelEnvironment = primaryScene.getSceneLevelsToRender().contains(GDXSceneLevel.MODEL);
         if (ImGui.checkbox(labels.get("Render Model Environment"), renderingModelEnvironment))
         {
            if (renderingModelEnvironment)
               primaryScene.getSceneLevelsToRender().remove(GDXSceneLevel.MODEL);
            else
               primaryScene.getSceneLevelsToRender().add(GDXSceneLevel.MODEL);
         }
         boolean renderingVirtualEnvironment = primaryScene.getSceneLevelsToRender().contains(GDXSceneLevel.VIRTUAL);
         if (ImGui.checkbox(labels.get("Render Virtual Environment"), renderingVirtualEnvironment))
         {
            if (renderingVirtualEnvironment)
               primaryScene.getSceneLevelsToRender().remove(GDXSceneLevel.VIRTUAL);
            else
               primaryScene.getSceneLevelsToRender().add(GDXSceneLevel.VIRTUAL);
         }
         ImGui.separator();
         if (ImGui.checkbox(labels.get("Middle-click view orbit"), middleClickOrbit))
         {
            setUseMiddleClickViewOrbit(middleClickOrbit.get());
         }
         if (ImGuiTools.volatileInputFloat(labels.get("Background shade"), backgroundShade))
         {
            setBackgroundShade(backgroundShade.get());
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
      ImGui.sameLine(ImGui.getWindowSizeX() - 220.0f);
      fpsCalculator.ping();
      String fpsString = String.valueOf((int) fpsCalculator.getFrequency());
      while (fpsString.length() < 3)
      {
         fpsString = " " + fpsString;
      }
      ImGui.text(fpsString + " Hz");
      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");
      ImGui.sameLine(ImGui.getWindowSizeX() - 100.0f);
      vrManager.renderImGuiEnableWidget();
      ImGui.endMainMenuBar();
   }

   private void saveApplicationSettings(ImGuiConfigurationLocation saveConfigurationLocation)
   {
      imGuiWindowAndDockSystem.saveConfiguration(saveConfigurationLocation);
      Consumer<ObjectNode> rootConsumer = root ->
      {
         root.put("windowWidth", Gdx.graphics.getWidth());
         root.put("windowHeight", Gdx.graphics.getHeight());
      };
      if (saveConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL)
      {
         LogTools.info("Saving libGDX settings to {}", libGDXSettingsFile.getWorkspaceFile().toString());
         JSONFileTools.save(libGDXSettingsFile.getWorkspaceFile(), rootConsumer);
      }
      else
      {
         LogTools.info("Saving libGDX settings to {}", libGDXSettingsFile.getExternalFile().toString());
         JSONFileTools.save(libGDXSettingsFile.getExternalFile(), rootConsumer);
      }
   }

   public void dispose()
   {
      imGuiWindowAndDockSystem.dispose();
      vrManager.dispose();
      primaryScene.dispose();
   }

   public void add3DPanel(GDX3DPanel panel3D)
   {
      panel3D.create(GDXInputMode.ImGui, glProfiler, primaryScene);
      panel3D.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      imGuiWindowAndDockSystem.getPanelManager().addPanel(panel3D.getImGuiPanel());
      additional3DPanels.add(panel3D);
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   /** TODO: Implement status bar */
   public void setStatus(String statusText)
   {
      this.statusText = statusText;
   }

   public void setVsync(boolean enabled)
   {
      vsync.set(enabled);
      Gdx.graphics.setVSync(enabled);
   }

   public void setForegroundFPS(int foregroundFPS)
   {
      this.foregroundFPS.set(foregroundFPS);
      Gdx.graphics.setForegroundFPS(foregroundFPS);
   }

   public ImGuiPanelManager getImGuiPanelManager()
   {
      return imGuiWindowAndDockSystem.getPanelManager();
   }

   public GDXImGuiPerspectiveManager getPerspectiveManager()
   {
      return perspectiveManager;
   }

   public GDX3DScene getPrimaryScene()
   {
      return primaryScene;
   }

   public GDX3DPanel getPrimary3DPanel()
   {
      return primary3DPanel;
   }

   public GDXVRManager getVRManager()
   {
      return vrManager;
   }

   public GDXImGuiWindowAndDockSystem getImGuiWindowAndDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }

   public HybridDirectory getConfigurationBaseDirectory()
   {
      return configurationBaseDirectory;
   }

   public long getRenderIndex()
   {
      return renderIndex;
   }

   public void setUseMiddleClickViewOrbit(boolean useMiddleClickViewOrbit)
   {
      this.middleClickOrbit.set(useMiddleClickViewOrbit);
      primary3DPanel.getCamera3D().setUseMiddleClickViewOrbit(useMiddleClickViewOrbit);
      for (GDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.getCamera3D().setUseMiddleClickViewOrbit(useMiddleClickViewOrbit);
      }
   }

   public void setBackgroundShade(float backgroundShade)
   {
      this.backgroundShade.set(backgroundShade);
      primary3DPanel.setBackgroundShade(backgroundShade);
      for (GDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.setBackgroundShade(backgroundShade);
      }
   }
}
