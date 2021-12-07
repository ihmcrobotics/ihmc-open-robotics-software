package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

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

   private final GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private final GDXVRManager vrManager = new GDXVRManager();
   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
//   private final GDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridDirectory perspectiveDirectory;
   private HybridFile libGDXSettingsFile;
   private final Stopwatch runTime = new Stopwatch().start();
   private String statusText = ""; // TODO: Add status at bottom of window
   private final ImGuiPanelSizeHandler view3DPanelSizeHandler = new ImGuiPanelSizeHandler();
   private ImGui3DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewInputProcessors = new ArrayList<>();
   private GLFrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;
   private final ImInt foregroundFPS = new ImInt(240);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImBoolean shadows = new ImBoolean(false);
   private final ImInt libGDXLogLevel = new ImInt(GDXTools.toGDX(LogTools.getLevel()));
   private final GDXImGuiPerspectiveManager perspectiveManager;

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
                                                          configurationBaseDirectory,
      updatedPerspectiveDirectory ->
      {
         libGDXSettingsFile = new HybridFile(updatedPerspectiveDirectory, "GDXSettings.json");
         imGuiWindowAndDockSystem.setDirectory(updatedPerspectiveDirectory);
      },
      loadWithDefaultMode ->
      {
         imGuiWindowAndDockSystem.loadConfiguration(loadWithDefaultMode);
         Path libGDXFile = loadWithDefaultMode ? libGDXSettingsFile.getWorkspaceFile() : libGDXSettingsFile.getExternalFile();
         JSONFileTools.load(libGDXFile, jsonNode ->
         {
            int width = jsonNode.get("windowWidth").asInt();
            int height = jsonNode.get("windowHeight").asInt();
            Gdx.graphics.setWindowedMode(width, height);
         });
      },
      saveWithDefaultMode ->
      {
         saveApplicationSettings(saveWithDefaultMode);
      });

//      guiRecorder = new GDXLinuxGUIRecorder(24, 0.8f, getClass().getSimpleName());
//      onCloseRequestListeners.add(guiRecorder::stop);
//      Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

      if (RECORD_VIDEO)
      {
         //         ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
//         ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
      }

      imGuiWindowAndDockSystem.getPanelManager().addPrimaryPanel(VIEW_3D_WINDOW_NAME);
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
      LogTools.info("Creating...");
      GDXTools.printGLVersion();

      sceneManager.create(GDXInputMode.ImGui);
      inputCalculator = new ImGui3DViewInput(sceneManager.getCamera3D(), this::getViewportSizeX, this::getViewportSizeY);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?
      imgui3DViewInputProcessors.add(sceneManager.getCamera3D()::processImGuiInput);

      double isoZoomOut = 7.0;
      sceneManager.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      sceneManager.addCoordinateFrame(0.3);

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));

      sceneManager.addRenderableProvider(vrManager::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
      addImGui3DViewInputProcessor(vrManager::process3DViewInput);
   }

   public void renderBeforeOnScreenUI()
   {
      vrManager.pollEventsAndRender(this, sceneManager);
      Gdx.graphics.setTitle(windowTitle + " - " + Gdx.graphics.getFramesPerSecond() + " FPS");
      imGuiWindowAndDockSystem.beforeWindowManagement();
      render3DView();
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();
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
         if (ImGui.inputInt("Foreground FPS", foregroundFPS, 1))
         {
            Gdx.graphics.setForegroundFPS(foregroundFPS.get());
         }
         if (ImGui.checkbox("Vsync", vsync))
         {
            Gdx.graphics.setVSync(vsync.get());
         }
         if (ImGui.checkbox("Shadows", shadows))
         {
            sceneManager.getSceneBasics().setShadowsEnabled(shadows.get());
         }
         if (ImGui.inputInt("libGDX log level", libGDXLogLevel, 1))
         {
            Gdx.app.setLogLevel(libGDXLogLevel.get());
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
      ImGui.sameLine(ImGui.getWindowSizeX() - 170.0f);
      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");
      ImGui.sameLine(ImGui.getWindowSizeX() - 100.0f);
      vrManager.renderImGuiEnableWidget();
      ImGui.endMainMenuBar();
   }

   private void render3DView()
   {
      view3DPanelSizeHandler.handleSizeBeforeBegin();
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
      int flags = ImGuiWindowFlags.None;
      //      flags |= ImGuiWindowFlags.NoDecoration;
      //      flags |= ImGuiWindowFlags.NoBackground;
      //      flags |= ImGuiWindowFlags.NoDocking;
      //      flags |= ImGuiWindowFlags.MenuBar;
      //      flags |= ImGuiWindowFlags.NoTitleBar;
      //      flags |= ImGuiWindowFlags.NoMouseInputs;
      ImGui.begin(VIEW_3D_WINDOW_NAME, flags);
      view3DPanelSizeHandler.handleSizeAfterBegin();

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
      sizeX = ImGui.getWindowSizeX();
      sizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
      float renderSizeX = sizeX * ANTI_ALIASING;
      float renderSizeY = sizeY * ANTI_ALIASING;

      inputCalculator.compute();
      for (Consumer<ImGui3DViewInput> imGuiInputProcessor : imgui3DViewInputProcessors)
      {
         imGuiInputProcessor.accept(inputCalculator);
      }

      // Allows for dynamically resizing the 3D view panel. Grows by 2x when needed, but never shrinks.
      if (frameBuffer == null || frameBuffer.getWidth() < renderSizeX || frameBuffer.getHeight() < renderSizeY)
      {
         if (frameBuffer != null)
            frameBuffer.dispose();

         int newWidth = frameBuffer == null ? Gdx.graphics.getWidth() * ANTI_ALIASING : frameBuffer.getWidth() * 2;
         int newHeight = frameBuffer == null ? Gdx.graphics.getHeight() * ANTI_ALIASING : frameBuffer.getHeight() * 2;
         LogTools.info("Allocating framebuffer of size: {}x{}", newWidth, newHeight);
         GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(newWidth, newHeight);
         frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
         frameBufferBuilder.addBasicStencilDepthPackedRenderBuffer();
         frameBuffer = frameBufferBuilder.build();
      }

      sceneManager.setViewportBounds(0, 0, (int) renderSizeX, (int) renderSizeY);
      sceneManager.renderShadowMap(Gdx.graphics.getWidth() * ANTI_ALIASING, Gdx.graphics.getHeight() * ANTI_ALIASING);

      frameBuffer.begin();
      sceneManager.render();
      frameBuffer.end();

      int frameBufferWidth = frameBuffer.getWidth();
      int frameBufferHeight = frameBuffer.getHeight();
      float percentOfFramebufferUsedX = renderSizeX / frameBufferWidth;
      float percentOfFramebufferUsedY = renderSizeY / frameBufferHeight;
      int textureID = frameBuffer.getColorBufferTexture().getTextureObjectHandle();
      float pMinX = posX;
      float pMinY = posY;
      float pMaxX = posX + sizeX;
      float pMaxY = posY + sizeY;
      float uvMinX = 0.0f;
      float uvMinY = percentOfFramebufferUsedY; // flip Y
      float uvMaxX = percentOfFramebufferUsedX;
      float uvMaxY = 0.0f;
      ImGui.getWindowDrawList().addImage(textureID, pMinX, pMinY, pMaxX, pMaxY, uvMinX, uvMinY, uvMaxX, uvMaxY);

      ImGui.end();
      ImGui.popStyleVar();
   }

   private void saveApplicationSettings(boolean saveDefault)
   {
      imGuiWindowAndDockSystem.saveConfiguration(saveDefault);
      Consumer<ObjectNode> rootConsumer = root ->
      {
         root.put("windowWidth", Gdx.graphics.getWidth());
         root.put("windowHeight", Gdx.graphics.getHeight());
      };
      if (saveDefault)
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
      sceneManager.dispose();
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   public void addImGui3DViewInputProcessor(Consumer<ImGui3DViewInput> processImGuiInput)
   {
      imgui3DViewInputProcessors.add(processImGuiInput);
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

   public GDX3DSceneManager get3DSceneManager()
   {
      return sceneManager;
   }

   public float getViewportSizeX()
   {
      return sizeX;
   }

   public float getViewportSizeY()
   {
      return sizeY;
   }

   public GDXVRManager getVRManager()
   {
      return vrManager;
   }

   public GDXImGuiWindowAndDockSystem getImGuiWindowAndDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }
}
