package us.ihmc.rdx.ui;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiTableFlags;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXKeyBindings;
import us.ihmc.rdx.RDXSettings;
import us.ihmc.rdx.imgui.ImGuiFrequencyDisplay;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.imgui.RDXPanelManager;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.vr.RDXVRManager;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridResourceDirectory;

import java.io.IOException;
import java.util.ArrayList;

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
 *     renderImGuiWidgets()
 *     calculate3DViewPick(ImGui3DViewInput)
 *     process3DViewInput(ImGui3DViewInput)
 *     getRenderables()
 *     renderTooltipsAndContextMenus()
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

   private static RDXBaseUI instance;

   public static RDXBaseUI getInstance()
   {
      return instance;
   }

   private GLProfiler glProfiler;
   private RDXSettings settings;

   private final RDX3DScene primaryScene = new RDX3DScene();
   private final RDX3DPanel primary3DPanel;
   private final ArrayList<RDX3DPanel> additional3DPanels = new ArrayList<>();
   private final RDXVRManager vrManager = new RDXVRManager();
   private final RDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
//   private final RDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private String configurationExtraPath;
   private final HybridResourceDirectory configurationBaseDirectory;
   private final ImGuiFrequencyDisplay frameRateDisplay = new ImGuiFrequencyDisplay("frameRateDisplay");
   private final Stopwatch runTime = new Stopwatch().start();
   private String statusText = ""; // TODO: Add status at bottom of window
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt foregroundFPSLimit = new ImInt(240);
   private final ImBoolean plotFrameRate = new ImBoolean(false);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImBoolean middleClickOrbit = new ImBoolean(false);
   private final ImBoolean modelSceneMouseCollisionEnabled = new ImBoolean(false);
   private final ImDouble view3DBackgroundShade = new ImDouble(RDX3DSceneTools.CLEAR_COLOR);
   private final ImInt libGDXLogLevel = new ImInt(LibGDXTools.toLibGDX(LogTools.getLevel()));
   private final ImDouble imguiFontScale = new ImDouble(1.0);
   private final RDXImGuiLayoutManager layoutManager;
   private final RDXKeyBindings keyBindings = new RDXKeyBindings();
   private long renderIndex = 0;
   private double isoZoomOut = 0.7;
   private enum Theme
   {
      LIGHT, DARK, CLASSIC
   }
   private Theme theme = Theme.LIGHT;
   private final String shadePrefix = "shade=";

   public RDXBaseUI()
   {
      this(0, null);
   }

   /**
    * @param windowTitle Title cased "My Window Title"; no symbols allowed
    */
   public RDXBaseUI(String windowTitle)
   {
      this(0, windowTitle);
   }

   /**
    * Typically you won't need this method. It's package private unless we find a use for it.
    *
    * @param additionalStackHeightForFindingCaller This is if you have something that sets up a RDXBaseUI for another class.
    *                                              We want the highest level calling class to be the one used for loading resources.
    * @param windowTitle Title cased "My Window Title"; no symbols allowed
    */
   /* package private*/ RDXBaseUI(int additionalStackHeightForFindingCaller, String windowTitle)
   {
      instance = this;

      StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
      Class<?> classForLoading = ExceptionTools.handle(() -> Class.forName(stackTraceElements[3 + additionalStackHeightForFindingCaller].getClassName()),
                                                       DefaultExceptionHandler.RUNTIME_EXCEPTION);
      LogTools.info("Using class for loading resources: {}", classForLoading.getName());

      this.windowTitle = windowTitle = windowTitle == null ? classForLoading.getSimpleName() : windowTitle;

      loadSettings();

      configurationExtraPath = "configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridResourceDirectory(IHMCCommonPaths.DOT_IHMC_DIRECTORY, classForLoading, "/").resolve(configurationExtraPath);

      if (!configurationBaseDirectory.isWorkspaceFileAccessAvailable())
      {
         LogTools.warn("We won't be able to write files to version controlled resources, because we probably aren't in a workspace.");
      }

      layoutManager = new RDXImGuiLayoutManager(classForLoading, configurationExtraPath, configurationBaseDirectory);
      imGuiWindowAndDockSystem = new RDXImGuiWindowAndDockSystem(layoutManager);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      layoutManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      layoutManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         int width = imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getWidth();
         int height = imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getHeight();
         Gdx.graphics.setWindowedMode(width, height);
         int x = imGuiWindowAndDockSystem.getPrimaryWindowContentAreaPosition().getX();
         int y = imGuiWindowAndDockSystem.getPrimaryWindowContentAreaPosition().getY();
         ((Lwjgl3Graphics) Gdx.graphics).getWindow().setPosition(x, y);
         return true;
      });
      layoutManager.getSaveListeners().add(imGuiWindowAndDockSystem::saveConfiguration);
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
      primary3DPanel.setBackgroundShade((float) view3DBackgroundShade.get());
   }

   /**
    * Create and load RDXSettings from disk.
    * If it can't load the settings from disk, the default settings will be used.
    * synchronized because we don't want to ever run this twice at the same time.
    */
   public synchronized void loadSettings()
   {
      settings = new RDXSettings();
      try
      {
         settings.load();
      }
      catch (IOException e)
      {
         LogTools.error("Unable to load RDXSettings.ini", e);
      }
      plotFrameRate.set(settings.plotFrameRateEnabled());
      vsync.set(settings.vsyncEnabled());
      foregroundFPSLimit.set(settings.getForegroundFPSLimit());
      libGDXLogLevel.set(settings.getLibGDXLogLevel());
      imguiFontScale.set(settings.getImguiFontScale());
      try
      {
         theme = Theme.valueOf(settings.getThemeName());
      }
      catch (IllegalArgumentException e)
      {
         LogTools.error("Invalid theme name in RDXSettings.ini");
      }
      view3DBackgroundShade.set(settings.getView3DBackgroundShade());
   }

   /**
    * Launches an RDX application using the specified adapter.
    * Will block until a shutdown is requested.
    *
    * @param applicationAdapter the adapter for the application
    */
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
                                                    applicationAdapter);
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
      imGuiWindowAndDockSystem.getPanelManager().addPanel(primary3DPanel);
      primary3DPanel.getIsShowing().set(true);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?

      primary3DPanel.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      primaryScene.addCoordinateFrame(0.3);

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());
      setTheme(theme); // TODO: move theme stuff to RDXImGuiWindowAndDockSystem?
      ImGui.getIO().setFontGlobalScale((float) imguiFontScale.get());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));

      vrManager.create();
      primaryScene.addRenderableProvider(vrManager::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
      primary3DPanel.addImGui3DViewPickCalculator(vrManager::calculate3DViewPick);
      primary3DPanel.addImGui3DViewInputProcessor(vrManager::process3DViewInput);
//      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Thread Debugger", vrManager::renderImGuiDebugWidgets);
//      imGuiWindowAndDockSystem.getPanelManager().addPanel("VR Settings", vrManager::renderImGuiTunerWidgets);

      keyBindings.register("Show key bindings", "Tab");
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

      keyBindings.renderKeyBindingsTable();
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
         if (ImGui.beginTable("settingsTable", 2, ImGuiTableFlags.None))
         {
            // First row (libgdx log level)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text("LibGDX log level: ");
            ImGui.tableSetColumnIndex(1);
            for (int i = 0; i <= Application.LOG_DEBUG; i++)
            {
               String logLevelName = "";
               switch (i)
               {
                  case 0 -> logLevelName = "None";
                  case 1 -> logLevelName = "Error";
                  case 2 -> logLevelName = "Info";
                  case 3 -> logLevelName = "Debug";
               }
               if (ImGui.radioButton(labels.get(logLevelName), Gdx.app.getLogLevel() == i)) {
                  settings.setLibGDXLogLevel(i);
                  Gdx.app.setLogLevel(i);
               }
               ImGui.sameLine();
            }

            // Second row (theme)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text("Theme: ");
            ImGui.tableSetColumnIndex(1);
            for (int i = 0; i < Theme.values().length; i++)
            {
               if (ImGui.radioButton(labels.get(StringUtils.capitalize(Theme.values()[i].name().toLowerCase())), this.theme == Theme.values()[i])) {
                  Theme newTheme = Theme.values()[i];
                  setTheme(newTheme);
               }
               if (i < Theme.values().length - 1)
                  ImGui.sameLine();
            }

            // Third row (background shade)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text("Background shade: ");
            ImGui.tableSetColumnIndex(1);
            if (ImGuiTools.sliderDouble(labels.get("##view3DBackgroundShadeSlider"), view3DBackgroundShade, 0.0f, 1.0f)) {
               setView3DBackgroundShade((float) view3DBackgroundShade.get());
            }
            // Only update the settings (which saves to disk) after you've let go of the slider
            if (ImGui.isItemDeactivatedAfterEdit())
            {
               settings.setView3DBackgroundShade((float) view3DBackgroundShade.get());
            }

            ImGui.sameLine();
            if (ImGui.button("Reset"))
            {
               view3DBackgroundShade.set(RDX3DSceneTools.CLEAR_COLOR);
               settings.setView3DBackgroundShade((float) view3DBackgroundShade.get());
               setView3DBackgroundShade((float) view3DBackgroundShade.get());
            }

            // Fourth row (foreground FPS limit)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text("FPS limit: ");
            // If vsync is enabled, disable the FPS limit slider
            if (vsync.get())
            {
               ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
               ImGui.pushStyleVar(ImGuiStyleVar.Alpha, ImGui.getStyle().getAlpha() * 0.5f);
            }
            ImGui.tableSetColumnIndex(1);
            if (ImGuiTools.sliderInt(labels.get("##foregroundFPSLimitSlider"), foregroundFPSLimit, 15, 240)) {
               settings.setForegroundFPSLimit(foregroundFPSLimit.get());
               Gdx.graphics.setForegroundFPS(foregroundFPSLimit.get());
            }
            if (vsync.get())
            {
               ImGui.popStyleVar();
               ImGui.popItemFlag();
            }

            // Fifth row (font scale)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text("Font scale: ");
            ImGui.tableSetColumnIndex(1);
            if (ImGuiTools.sliderDouble(labels.get("##imguiFontScaleSlider"), imguiFontScale, 1.0, 2.0, "%.1f")) {
               settings.setImguiFontScale(imguiFontScale.get());
            }
            // Change the font scale after you've let go of the slider
            if (ImGui.isItemDeactivatedAfterEdit())
            {
               ImGui.getIO().setFontGlobalScale((float) imguiFontScale.get());
            }

            ImGui.endTable();
         }

         /* Start checkbox settings */
         if (ImGui.checkbox(labels.get("Frame rate plot"), plotFrameRate))
         {
            settings.setPlotFrameRate(plotFrameRate.get());
         }
         if (ImGui.checkbox(labels.get("Vsync"), vsync))
         {
            settings.setVsync(vsync.get());
            Gdx.graphics.setForegroundFPS(Integer.MAX_VALUE);
            Gdx.graphics.setVSync(vsync.get());
         }

         ImGui.separator(); // Environment section
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
         ImGui.separator(); // Mouse behavior section
         if (ImGui.checkbox(labels.get("Model scene mouse collision enabled"), modelSceneMouseCollisionEnabled))
         {
            setModelSceneMouseCollisionEnabled(modelSceneMouseCollisionEnabled.get());
         }
         if (ImGui.checkbox(labels.get("Middle-click view orbit"), middleClickOrbit))
         {
            setUseMiddleClickViewOrbit(middleClickOrbit.get());
         }
         /* End checkbox settings */

         if (ImGui.menuItem("Show key bindings..."))
         {
            keyBindings.showKeybindings();
         }

         ImGui.endMenu();
      }

      vrManager.renderMenuBar();

      frameRateDisplay.ping();

      if (plotFrameRate.get())
      {
         // Currently we manually tune this value when we change the stuff in the status a
         float menuBarStatusWidth = 212.0f;
         ImGui.sameLine(ImGui.getWindowSizeX() - menuBarStatusWidth);
         frameRateDisplay.renderPlot();
      }
      else
      {
         float menuBarStatusWidth = 110.0f;
         ImGui.sameLine(ImGui.getWindowSizeX() - menuBarStatusWidth);
      }

      frameRateDisplay.renderHz();

      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");

      ImGui.endMainMenuBar();
   }

   public void dispose()
   {
      imGuiWindowAndDockSystem.dispose();
      vrManager.dispose();
      primaryScene.dispose();

      instance = null;
   }

   public void add3DPanel(RDX3DPanel panel3D)
   {
      add3DPanel(panel3D, primaryScene);
   }

   public void add3DPanel(RDX3DPanel panel3D, RDX3DScene scene3D)
   {
      panel3D.create(RDXInputMode.ImGui, glProfiler, scene3D);
      panel3D.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      imGuiWindowAndDockSystem.getPanelManager().addPanel(panel3D);
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

   public void setForegroundFPSLimit(int foregroundFPSLimit)
   {
      this.foregroundFPSLimit.set(foregroundFPSLimit);
      Gdx.graphics.setForegroundFPS(foregroundFPSLimit);
   }

   public RDXPanelManager getImGuiPanelManager()
   {
      return imGuiWindowAndDockSystem.getPanelManager();
   }

   public RDXImGuiLayoutManager getLayoutManager()
   {
      return layoutManager;
   }

   public RDXKeyBindings getKeyBindings()
   {
      return keyBindings;
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

   public void setView3DBackgroundShade(float shade)
   {
      this.view3DBackgroundShade.set(shade);
      primary3DPanel.setBackgroundShade(shade);
      for (RDX3DPanel additional3DPanel : additional3DPanels)
      {
         additional3DPanel.setBackgroundShade(shade);
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
      settings.setThemeName(theme.name());
      switch (theme)
      {
         case LIGHT -> ImGui.styleColorsLight();
         case DARK -> ImGui.styleColorsDark();
         case CLASSIC -> ImGui.styleColorsClassic();
      }
      this.theme = theme;
   }

   public static void pushNotification(String text)
   {
      instance.getPrimary3DPanel().getNotificationManager().pushNotification(2, text);
   }
}