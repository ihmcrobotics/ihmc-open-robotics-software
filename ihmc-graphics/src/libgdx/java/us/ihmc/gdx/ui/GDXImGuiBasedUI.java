package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
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
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.FileVisitResult;
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
//   private final GDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;
   private final String windowTitle;
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridDirectory perspectiveDirectory;
   private HybridFile libGDXSettingsFile;
   private final Stopwatch runTime = new Stopwatch().start();
   private String statusText = "";
   private final ImGuiPanelSizeHandler view3DPanelSizeHandler = new ImGuiPanelSizeHandler();
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
   private boolean needToReindexPerspectives = true;
   private final ImString perspectiveNameToSave = new ImString("", 100);
   private final ImBoolean perspectiveDefaultMode = new ImBoolean(false);
   private final ArrayList<String> perspectives = new ArrayList<>();
   private String currentPerspective = "Main";

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

      configurationExtraPath = "/configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridDirectory(dotIHMCDirectory,
                                                       directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForLoading,
                                                       configurationExtraPath);

      imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem();
      applyPerspectiveDirectory();

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
      AtomicReference<Double> windowWidth = new AtomicReference<>((double) 800);
      AtomicReference<Double> windowHeight = new AtomicReference<>((double) 600);
      JSONFileTools.loadUserWithClasspathDefaultFallback(libGDXSettingsFile, jsonNode ->
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
      render3DView();
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();

      if (GDXVRManager.isVREnabled())
         vrManager.render(sceneManager);
   }

   private void renderMenuBar()
   {
      if (needToReindexPerspectives)
      {
         needToReindexPerspectives = false;
         Path directory = perspectiveDefaultMode.get() ? configurationBaseDirectory.getWorkspaceDirectory() : configurationBaseDirectory.getExternalDirectory();
         perspectives.clear();
         perspectives.add("Main");
         PathTools.walkFlat(directory, new BasicPathVisitor()
         {
            @Override
            public FileVisitResult visitPath(Path path, PathType pathType)
            {
               if (pathType == PathType.DIRECTORY)
               {
                  String directoryName = path.getFileName().toString();
                  String matchString = "Perspective";
                  if (directoryName.endsWith(matchString))
                  {
                     String perspectiveName = directoryName.substring(0, directoryName.lastIndexOf(matchString));
                     LogTools.info("Found perspective {}", perspectiveName);
                     perspectives.add(perspectiveName);
                  }
               }
               return FileVisitResult.CONTINUE;
            }
         });
      }

      ImGui.beginMainMenuBar();
      if (ImGui.beginMenu("Perspective"))
      {
         for (String perspective : perspectives)
         {
            if (ImGui.radioButton(perspective, currentPerspective.equals(perspective)))
            {
               currentPerspective = perspective;
               applyPerspectiveDirectory();
               imGuiWindowAndDockSystem.loadConfiguration(perspectiveDefaultMode.get());
            }
            if (currentPerspective.equals(perspective))
            {
               ImGui.sameLine();
               if (ImGui.button("Save"))
               {
                  saveApplicationSettings(perspectiveDefaultMode.get());
               }
            }
         }

         ImGui.text("New:");
         ImGui.sameLine();
         ImGui.inputText("###", perspectiveNameToSave , ImGuiInputTextFlags.CallbackResize);
         String perpectiveNameToCreateString = perspectiveNameToSave.get();
         if (!perpectiveNameToCreateString.isEmpty())
         {
            ImGui.sameLine();
            if (ImGui.button("Create"))
            {
               String sanitizedName = perpectiveNameToCreateString.replaceAll(" ", "");
               perspectives.add(sanitizedName);
               currentPerspective = sanitizedName;
               applyPerspectiveDirectory();
               perspectiveNameToSave.clear();
            }
         }

//         if (ImGui.button("Save"))
//         {
//            saveApplicationSettings(perspectiveDefaultMode.get());
//         }
//         ImGui.sameLine();
//         if (ImGui.button("Load"))
//         {
//            imGuiWindowAndDockSystem.loadConfiguration(perspectiveDefaultMode.get());
//         }

         ImGui.separator();
         ImGui.text("Save location:");
         if (ImGui.radioButton("User home###PerspectiveUserHomeMode", !perspectiveDefaultMode.get()))
         {
            perspectiveDefaultMode.set(false);
            needToReindexPerspectives = true;
         }
         ImGui.sameLine();
         if (ImGui.radioButton("Version control###PerspectiveDefaultMode", perspectiveDefaultMode.get()))
         {
            perspectiveDefaultMode.set(true);
            needToReindexPerspectives = true;
         }
         ImGui.endMenu();
      }
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
   }

   private void applyPerspectiveDirectory()
   {
      perspectiveDirectory = new HybridDirectory(dotIHMCDirectory,
                                                 directoryNameToAssumePresent,
                                                 subsequentPathToResourceFolder,
                                                 classForLoading,
                                                 configurationExtraPath + (currentPerspective.equals("Main") ? "" : "/" + currentPerspective + "Perspective"));
      libGDXSettingsFile = new HybridFile(perspectiveDirectory, "GDXSettings.json");
      imGuiWindowAndDockSystem.setDirectory(perspectiveDirectory);
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
}
