package us.ihmc.rdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.imgui.ImGuiPanelManager;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.vr.RDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkingDirectoryPathComponents;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Method call order:
 *
 * <pre>
 * create()
 *
 * loop:
 *     update()
 *     calculateVRPick(RDXVRContext)
 *     processVRInput(RDXVRContext)
 *     calculate3DViewPick(ImGui3DViewInput)
 *     process3DViewInput(ImGui3DViewInput)
 *     renderImGuiWidgets()
 *     renderTooltipsAndContextMenus()
 *     getRenderables()
 *
 * destroy()
 * </pre>
 *
 * TODO: Is there a better theoretical order? Potentially doing processInput first, then update, then render.
 *
 * It is recommended that in every class, the methods that exist in them mapping to the above are defined in the class in that order.
 *
 * Additionally, the update() is only called first if you code according to this in the Lwjgl3ApplicationAdapter provided to RDXBaseUI:
 *
 * <pre>
 * &#64;Override
 * public void render()
 * {
 *    // call update() methods here
 *
 *    baseUI.renderBeforeOnScreenUI();
 *    baseUI.renderEnd();
 * }
 * </pre>
 *
 */
public class RDXBaseUI
{
   public static final int ANTI_ALIASING = 2;

   private static boolean RECORD_VIDEO = Boolean.parseBoolean(System.getProperty("record.video"));
   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active
   private static final String VIEW_3D_WINDOW_NAME = "3D View";

   private GLProfiler glProfiler;
   private final RDX3DScene primaryScene = new RDX3DScene();
   private final RDX3DPanel primary3DPanel;
   private final ArrayList<RDX3DPanel> additional3DPanels = new ArrayList<>();
   private final RDXVRManager vrManager = new RDXVRManager();
   private final RDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
//   private final RDXLinuxGUIRecorder guiRecorder;
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
   private final ImBoolean modelSceneMouseCollisionEnabled = new ImBoolean(false);
   private final ImFloat backgroundShade = new ImFloat(RDX3DSceneTools.CLEAR_COLOR);
   private final ImInt libGDXLogLevel = new ImInt(LibGDXTools.toLibGDX(LogTools.getLevel()));
   private final ImFloat imguiFontScale = new ImFloat(1.0f);
   private final RDXImGuiLayoutManager layoutManager;
   private long renderIndex = 0;
   private double isoZoomOut = 0.7;
   private enum Theme
   {
      LIGHT, DARK, CLASSIC
   }
   private Theme theme = Theme.LIGHT;
   private Path themeFilePath;
   private final String shadePrefix = "shade=";

   public RDXBaseUI()
   {
      this(0, null, null, null);
   }

   public RDXBaseUI(String windowTitle)
   {
      this(0, null, null, windowTitle);
   }

   /**
    * Typically you won't need this method. It's package private unless we find a use for it.
    *
    * @param additionalStackHeightForFindingCaller This is if you have something that sets up a RDXBaseUI for another class.
    *                                              We want the highest level calling class to be the one used for loading resources.
    * @param directoryNameToAssumePresent Directory that's either a part present in the working directory of an immediate child.
    * @param subsequentPathToResourceFolder The subsequent path to the resources folder from the directory assumed to be present.
    */
   /* package private*/ RDXBaseUI(int additionalStackHeightForFindingCaller,
                                  String directoryNameToAssumePresent,
                                  String subsequentPathToResourceFolder,
                                  String windowTitle)
   {
      StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
      Class<?> classForLoading = ExceptionTools.handle(() -> Class.forName(stackTraceElements[3 + additionalStackHeightForFindingCaller].getClassName()),
                                                       DefaultExceptionHandler.RUNTIME_EXCEPTION);
      LogTools.info("Using class for loading resources: {}", classForLoading.getName());

      this.windowTitle = windowTitle = windowTitle == null ? classForLoading.getSimpleName() : windowTitle;

      // Try to figure out where the resources for this class are
      if (directoryNameToAssumePresent == null || subsequentPathToResourceFolder == null)
      {
         WorkingDirectoryPathComponents inferredPathComponents = WorkspacePathTools.inferWorkingDirectoryPathComponents(classForLoading);
         if (inferredPathComponents != null)
         {
            directoryNameToAssumePresent = inferredPathComponents.getDirectoryNameToAssumePresent();
            subsequentPathToResourceFolder = inferredPathComponents.getSubsequentPathToResourceFolder();
         }
      }

      if (directoryNameToAssumePresent == null || subsequentPathToResourceFolder == null)
      {
         LogTools.warn("We won't be able to write files to version controlled resources, because we probably aren't in a workspace.");
      }

      configurationExtraPath = "/configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridDirectory(dotIHMCDirectory,
                                                       directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForLoading,
                                                       configurationExtraPath);

      imGuiWindowAndDockSystem = new RDXImGuiWindowAndDockSystem();
      layoutManager = new RDXImGuiLayoutManager(classForLoading,
                                                directoryNameToAssumePresent,
                                                subsequentPathToResourceFolder,
                                                configurationExtraPath,
                                                configurationBaseDirectory);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(updatedLayoutDirectory ->
      {
         libGDXSettingsFile = new HybridFile(updatedLayoutDirectory, "GDXSettings.json");
      });
      layoutManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      layoutManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         Gdx.graphics.setWindowedMode(imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getWidth(),
                                      imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getHeight());
         ((Lwjgl3Graphics) Gdx.graphics).getWindow().setPosition(imGuiWindowAndDockSystem.getPrimaryWindowPosition().getX(),
                                                                 imGuiWindowAndDockSystem.getPrimaryWindowPosition().getY());
         return true;
      });
      layoutManager.getSaveListeners().add(this::saveApplicationSettings);
      layoutManager.applyLayoutDirectory();

//      guiRecorder = new RDXLinuxGUIRecorder(24, 0.8f, getClass().getSimpleName());
//      onCloseRequestListeners.add(guiRecorder::stop);
//      Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

      if (RECORD_VIDEO)
      {
         //         ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
//         ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
      }

      primary3DPanel = new RDX3DPanel(VIEW_3D_WINDOW_NAME, ANTI_ALIASING, true);
   }

   public void launchRDXApplication(Lwjgl3ApplicationAdapter applicationAdapter)
   {
      LogTools.info("Launching RDX application");
      // TODO: We could show a splash screen here until the app shows up
      Lwjgl3ApplicationConfiguration applicationConfiguration = LibGDXApplicationCreator.getDefaultConfiguration(windowTitle);
      // Hide the window at the beginning. If you don't do this, you get a window frame
      // with the contents behind the window displayed for a few seconds, which is really
      // consifusing and error-prone.
      applicationConfiguration.setInitialVisible(false);
      LibGDXApplicationCreator.launchGDXApplication(applicationConfiguration,
                                                    applicationAdapter,
                                                    windowTitle);
   }

   public void create()
   {
      create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXSceneLevel... sceneLevels)
   {
      LogTools.info("Creating...");
      LibGDXTools.printGLVersion();

      if (LibGDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = LibGDXTools.createGLProfiler();

      primaryScene.create(sceneLevels);
      primaryScene.addDefaultLighting();
      primary3DPanel.create(RDXInputMode.ImGui, glProfiler, primaryScene);
      imGuiWindowAndDockSystem.getPanelManager().addPanel(primary3DPanel.getImGuiPanel());
      primary3DPanel.getImGuiPanel().getIsShowing().set(true);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?

      primary3DPanel.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      primaryScene.addCoordinateFrame(0.3);

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle(), layoutManager);

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));

      vrManager.create();
      primaryScene.addRenderableProvider(vrManager::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
      primary3DPanel.addImGui3DViewPickCalculator(vrManager::calculate3DViewPick);
      primary3DPanel.addImGui3DViewInputProcessor(vrManager::process3DViewInput);
      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Thread Debugger", vrManager::renderImGuiDebugWidgets);
      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Settings", vrManager::renderImGuiTunerWidgets);

      themeFilePath = Paths.get(System.getProperty("user.home"), ".ihmc/themePreference.ini");
      if (Files.exists(themeFilePath))
      {
         List<String> lines = FileTools.readAllLines(themeFilePath, DefaultExceptionHandler.PROCEED_SILENTLY);
         int numberOfLines = lines.size();
         String firstLine = "";
         String secondLine = "";
         if (numberOfLines > 0)
            firstLine = lines.get(0);
         if (numberOfLines > 1)
            secondLine = lines.get(1);
         for (Theme theme : Theme.values())
            if (firstLine.contains(theme.name()))
               setTheme(theme);
         try
         {
            if (!secondLine.isEmpty())
            {
               backgroundShade.set(Float.parseFloat(secondLine.substring(shadePrefix.length())));
               setBackgroundShade(backgroundShade.get());
            }
         }
         catch (Exception ignored)
         {
         }
      }
   }

   public void renderBeforeOnScreenUI()
   {
      vrManager.pollEventsAndRender(this, primaryScene);
      Gdx.graphics.setTitle(windowTitle);
      imGuiWindowAndDockSystem.beforeWindowManagement();
      primary3DPanel.render();
      for (RDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.render();
      }
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();
      ++renderIndex;

      // Show the window now that it's been loaded; we started it while it wasn't visible.
      // Some heavyweight applications still will show an unrendered window after the first
      // render, so let's show it after the second render.
      if (renderIndex == 2)
      {
         ((Lwjgl3Graphics) Gdx.graphics).getWindow().setVisible(true);
      }
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
         boolean renderingGroundTruthEnvironment = primaryScene.getSceneLevelsToRender().contains(RDXSceneLevel.GROUND_TRUTH);
         if (ImGui.checkbox(labels.get("Render Ground Truth Environment"), renderingGroundTruthEnvironment))
         {
            if (renderingGroundTruthEnvironment)
               primaryScene.getSceneLevelsToRender().remove(RDXSceneLevel.GROUND_TRUTH);
            else
               primaryScene.getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
         }
         boolean renderingModelEnvironment = primaryScene.getSceneLevelsToRender().contains(RDXSceneLevel.MODEL);
         if (ImGui.checkbox(labels.get("Render Model Environment"), renderingModelEnvironment))
         {
            if (renderingModelEnvironment)
               primaryScene.getSceneLevelsToRender().remove(RDXSceneLevel.MODEL);
            else
               primaryScene.getSceneLevelsToRender().add(RDXSceneLevel.MODEL);
         }
         boolean renderingVirtualEnvironment = primaryScene.getSceneLevelsToRender().contains(RDXSceneLevel.VIRTUAL);
         if (ImGui.checkbox(labels.get("Render Virtual Environment"), renderingVirtualEnvironment))
         {
            if (renderingVirtualEnvironment)
               primaryScene.getSceneLevelsToRender().remove(RDXSceneLevel.VIRTUAL);
            else
               primaryScene.getSceneLevelsToRender().add(RDXSceneLevel.VIRTUAL);
         }
         if (ImGui.checkbox(labels.get("Model scene mouse collision enabled"), modelSceneMouseCollisionEnabled))
         {
            setModelSceneMouseCollisionEnabled(modelSceneMouseCollisionEnabled.get());
         }
         ImGui.separator();
         if (ImGui.checkbox(labels.get("Middle-click view orbit"), middleClickOrbit))
         {
            setUseMiddleClickViewOrbit(middleClickOrbit.get());
         }
         float previousShade = backgroundShade.get();
         if (ImGuiTools.volatileInputFloat(labels.get("Background shade"), backgroundShade))
         {
            setBackgroundShade(backgroundShade.get());
         }

         ImGui.separator();
         ImGui.text("UI Theme:");
         ImGui.sameLine();
         Theme previousTheme = theme;
         for (int i = 0; i < Theme.values().length; ++i)
         {
            if (ImGui.radioButton(labels.get(StringUtils.capitalize(Theme.values()[i].name().toLowerCase())), this.theme == Theme.values()[i]))
               setTheme(Theme.values()[i]);
            if (i < Theme.values().length - 1)
               ImGui.sameLine();
         }
         if (theme != previousTheme || previousShade != backgroundShade.get())
         {
            FileTools.ensureFileExists(themeFilePath, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            FileTools.writeAllLines(List.of("theme=" + theme.name() + "\n" + shadePrefix + backgroundShade),
                                    themeFilePath,
                                    WriteOption.TRUNCATE,
                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
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
      if (saveConfigurationLocation.isVersionControl())
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
      primary3DPanel.dispose();
      primaryScene.dispose();
   }

   public void add3DPanel(RDX3DPanel panel3D)
   {
      add3DPanel(panel3D, primaryScene);
   }

   public void add3DPanel(RDX3DPanel panel3D, RDX3DScene scene3D)
   {
      panel3D.create(RDXInputMode.ImGui, glProfiler, scene3D);
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

   public RDXImGuiLayoutManager getLayoutManager()
   {
      return layoutManager;
   }

   public RDX3DScene getPrimaryScene()
   {
      return primaryScene;
   }

   public RDX3DPanel getPrimary3DPanel()
   {
      return primary3DPanel;
   }

   public RDXVRManager getVRManager()
   {
      return vrManager;
   }

   public RDXImGuiWindowAndDockSystem getImGuiWindowAndDockSystem()
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
      for (RDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.getCamera3D().setUseMiddleClickViewOrbit(useMiddleClickViewOrbit);
      }
   }

   public void setBackgroundShade(float backgroundShade)
   {
      this.backgroundShade.set(backgroundShade);
      primary3DPanel.setBackgroundShade(backgroundShade);
      for (RDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.setBackgroundShade(backgroundShade);
      }
   }

   public void setModelSceneMouseCollisionEnabled(boolean modelSceneMouseCollisionEnabled)
   {
      this.modelSceneMouseCollisionEnabled.set(modelSceneMouseCollisionEnabled);
      primary3DPanel.setModelSceneMouseCollisionEnabled(modelSceneMouseCollisionEnabled);
      for (RDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.setModelSceneMouseCollisionEnabled(modelSceneMouseCollisionEnabled);
      }
   }

   public void setTheme(Theme theme)
   {
      switch (theme)
      {
         case LIGHT -> ImGui.styleColorsLight();
         case DARK -> ImGui.styleColorsDark();
         case CLASSIC -> ImGui.styleColorsClassic();
      }
      this.theme = theme;
   }

   public void showGroundTruthEnvironment(boolean show)
   {
      if (primaryScene.getSceneLevelsToRender() != null)
      {
         boolean rendering = primaryScene.getSceneLevelsToRender().contains(RDXSceneLevel.GROUND_TRUTH);
         if (show && !rendering)
            primaryScene.getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
         else if (!show && rendering)
            primaryScene.getSceneLevelsToRender().remove(RDXSceneLevel.GROUND_TRUTH);
      }
   }
}