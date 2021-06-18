package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiCond;
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
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class GDXImGuiBasedUI
{
   private static boolean RECORD_VIDEO = Boolean.parseBoolean(System.getProperty("record.video"));

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   private static final String VIEW_3D_WINDOW_NAME = "3D View";

   private final GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private final GDXVRManager vrManager = new GDXVRManager();

   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final ImGuiPanelManager imGuiPanelManager;

//   private final GDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private Path imGuiUserSettingsPath;
   private Path gdxUserSettingsPath;
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;
   private final Stopwatch runTime = new Stopwatch().start();
   private String statusText = "";

   private ImGui3DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui3DViewInput>> imGuiInputProcessors = new ArrayList<>();
   private boolean dragging = false;
   private float dragBucketX;
   private float dragBucketY;
   private GLFrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;

   private final ImInt foregroundFPS = new ImInt(240);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImInt libGDXLogLevel = new ImInt(GDXTools.toGDX(LogTools.getLevel()));

   public GDXImGuiBasedUI(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder, "");
   }

   public GDXImGuiBasedUI(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String windowTitle)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      this.windowTitle = windowTitle;

      imGuiUserSettingsPath
            = Paths.get(System.getProperty("user.home"), ".ihmc/" + windowTitle.replaceAll(" ", "") + "ImGuiSettings.ini").toAbsolutePath().normalize();
      gdxUserSettingsPath = imGuiUserSettingsPath.getParent()
                                                 .resolve(imGuiUserSettingsPath.getFileName().toString().replace("ImGuiSettings.ini", "GDXSettings.json"));
      imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem(classForLoading,
                                                                 directoryNameToAssumePresent,
                                                                 subsequentPathToResourceFolder,
                                                                 imGuiUserSettingsPath);
      imGuiPanelManager = new ImGuiPanelManager(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder);

//      guiRecorder = new GDXLinuxGUIRecorder(24, 0.8f, getClass().getSimpleName());
//      onCloseRequestListeners.add(guiRecorder::stop);
//      Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

      if (RECORD_VIDEO)
      {
         //         ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
//         ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
      }

      imGuiPanelManager.addPrimaryPanel(VIEW_3D_WINDOW_NAME);
   }

   public void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter)
   {
      AtomicReference<Double> windowWidth = new AtomicReference<>((double) 800);
      AtomicReference<Double> windowHeight = new AtomicReference<>((double) 600);
      JSONFileTools.loadWithClasspathDefault(gdxUserSettingsPath,
                                             classForLoading,
                                             directoryNameToAssumePresent,
                                             subsequentPathToResourceFolder,
                                             "/imgui",
                                             jsonNode ->
      {
         windowWidth.set(jsonNode.get("windowWidth").asDouble());
         windowHeight.set(jsonNode.get("windowHeight").asDouble());
      });

      LogTools.info("Launching GDX application");
      GDXApplicationCreator.launchGDXApplication(applicationAdapter, windowTitle, windowWidth.get(), windowHeight.get());
   }

   public void create()
   {
      LogTools.info("create()");

      sceneManager.create(GDXInputMode.ImGui);
      inputCalculator = new ImGui3DViewInput(sceneManager.getCamera3D(), this::getViewportSizeX, this::getViewportSizeY);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?
      imGuiInputProcessors.add(sceneManager.getCamera3D()::processImGuiInput);

      double isoZoomOut = 7.0;
      sceneManager.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);

      sceneManager.addCoordinateFrame(0.3);

      if (GDXVRManager.isVREnabled())
      {
         enableVR();
      }

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));
   }

   private void enableVR()
   {
      vrManager.create();
      sceneManager.addRenderableProvider(vrManager, GDXSceneLevel.VIRTUAL);
      addImGui3DViewInputProcessor(vrManager::process3DViewInput);
   }

   public void pollVREvents()
   {
      if (GDXVRManager.isVREnabled())
      {
         vrManager.pollEvents();
      }
   }

   public void renderBeforeOnScreenUI()
   {
      Gdx.graphics.setTitle(windowTitle + " - " + Gdx.graphics.getFramesPerSecond() + " FPS");
      GDX3DSceneTools.glClearGray(0.3f);
      imGuiWindowAndDockSystem.beforeWindowManagement();
      imGuiPanelManager.renderPanels();
   }

   public void renderEnd()
   {
      ImGui.beginMainMenuBar();
      if (ImGui.beginMenu("Window"))
      {
         if (ImGui.menuItem("Save Configuration to ~/.ihmc"))
         {
            saveApplicationSettings(false);
         }
         if (ImGui.menuItem("Save Configuration as Default"))
         {
            saveApplicationSettings(true);
         }
         if (ImGui.menuItem("Load Layout from ~/.ihmc"))
         {
            imGuiWindowAndDockSystem.loadLocalLayout();
         }
         if (ImGui.menuItem("Load Default Layout"))
         {
            imGuiWindowAndDockSystem.loadDefaultLayout();
         }
         ImGui.endMenu();
      }
      if (ImGui.beginMenu("Panels"))
      {
         imGuiPanelManager.renderPanelMenu();
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
      if (GDXVRManager.isVREnabled())
      {
         ImGui.text("VR Enabled");
      }
      else if (ImGui.button("Enable VR"))
      {
         enableVR();
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("It is recommended to start SteamVR and power on the VR controllers before clicking this button.");
      }
      ImGui.endMainMenuBar();

      ImGui.setNextWindowSize(800.0f, 600.0f, ImGuiCond.FirstUseEver);
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
      int flags = ImGuiWindowFlags.None;
//      flags |= ImGuiWindowFlags.NoDecoration;
//      flags |= ImGuiWindowFlags.NoBackground;
//      flags |= ImGuiWindowFlags.NoDocking;
//      flags |= ImGuiWindowFlags.MenuBar;
//      flags |= ImGuiWindowFlags.NoTitleBar;
//      flags |= ImGuiWindowFlags.NoMouseInputs;
      ImGui.begin(VIEW_3D_WINDOW_NAME, flags);

      int antiAliasing = 2;
      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
      sizeX = ImGui.getWindowSizeX();
      sizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
      float renderSizeX = sizeX * antiAliasing;
      float renderSizeY = sizeY * antiAliasing;

      inputCalculator.compute();
      for (Consumer<ImGui3DViewInput> imGuiInputProcessor : imGuiInputProcessors)
      {
         imGuiInputProcessor.accept(inputCalculator);
      }

      if (frameBuffer == null || frameBuffer.getWidth() < renderSizeX || frameBuffer.getHeight() < renderSizeY)
      {
         if (frameBuffer != null)
            frameBuffer.dispose();

         int newWidth = frameBuffer == null ? Gdx.graphics.getWidth() * antiAliasing : frameBuffer.getWidth() * 2;
         int newHeight = frameBuffer == null ? Gdx.graphics.getHeight() * antiAliasing : frameBuffer.getHeight() * 2;
         LogTools.info("Allocating framebuffer of size: {}x{}", newWidth, newHeight);
         GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(newWidth, newHeight);
         frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
         frameBufferBuilder.addBasicStencilDepthPackedRenderBuffer();
         frameBuffer = frameBufferBuilder.build();
      }

      frameBuffer.begin();
      sceneManager.setViewportBounds(0, 0, (int) renderSizeX, (int) renderSizeY);
      sceneManager.render();
      frameBuffer.end();

      float percentOfFramebufferUsedX = renderSizeX / frameBuffer.getWidth();
      float percentOfFramebufferUsedY = renderSizeY / frameBuffer.getHeight();
      int textureId = frameBuffer.getColorBufferTexture().getTextureObjectHandle();
      float pMinX = posX;
      float pMinY = posY;
      float pMaxX = posX + sizeX;
      float pMaxY = posY + sizeY;
      float uvMinX = 0.0f;
      float uvMinY = percentOfFramebufferUsedY; // flip Y
      float uvMaxX = percentOfFramebufferUsedX;
      float uvMaxY = 0.0f;
      ImGui.getWindowDrawList().addImage(textureId, pMinX, pMinY, pMaxX, pMaxY, uvMinX, uvMinY, uvMaxX, uvMaxY);

      ImGui.end();
      ImGui.popStyleVar();

      if (imGuiWindowAndDockSystem.isFirstRenderCall())
      {
         if (!Files.exists(imGuiUserSettingsPath))
         {
            // TODO: Load default settings file from resources
         }

         imGuiPanelManager.loadConfiguration(imGuiUserSettingsPath);
      }

      imGuiWindowAndDockSystem.afterWindowManagement();

      if (GDXVRManager.isVREnabled())
         vrManager.render(sceneManager);
   }

   private void saveApplicationSettings(boolean saveDefault)
   {
      imGuiWindowAndDockSystem.saveImGuiLayout(saveDefault);
      imGuiPanelManager.saveConfiguration(imGuiUserSettingsPath, saveDefault);
      Consumer<ObjectNode> rootConsumer = root ->
      {
         root.put("windowWidth", Gdx.graphics.getWidth());
         root.put("windowHeight", Gdx.graphics.getHeight());
      };
      String saveFileNameString = gdxUserSettingsPath.getFileName().toString();
      if (saveDefault)
      {
         JSONFileTools.saveToClasspath(directoryNameToAssumePresent, subsequentPathToResourceFolder, "imgui/" + saveFileNameString, rootConsumer);
      }
      else
      {
         LogTools.info("Saving libGDX settings to {}", gdxUserSettingsPath.toString());
         JSONFileTools.save(gdxUserSettingsPath, rootConsumer);
      }
   }

   public void dispose()
   {
      imGuiWindowAndDockSystem.dispose();
      if (GDXVRManager.isVREnabled())
         vrManager.dispose();
      sceneManager.dispose();
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   public void addImGui3DViewInputProcessor(Consumer<ImGui3DViewInput> processImGuiInput)
   {
      imGuiInputProcessors.add(processImGuiInput);
   }

   public void setStatus(String statusText)
   {
      this.statusText = statusText;
   }

   public ImGuiPanelManager getImGuiPanelManager()
   {
      return imGuiPanelManager;
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
}
