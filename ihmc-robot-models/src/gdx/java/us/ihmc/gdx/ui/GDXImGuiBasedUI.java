package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;

import java.nio.file.Files;
import java.util.ArrayList;
import java.util.function.Consumer;

public class GDXImGuiBasedUI
{
   private static boolean RECORD_VIDEO = Boolean.parseBoolean(System.getProperty("record.video"));
   private static boolean ENABLE_VR = Boolean.parseBoolean(System.getProperty("enable.vr"));

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   private static final String VIEW_3D_WINDOW_NAME = "3D View";

   private final GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private final GDXVRManager vrManager = ENABLE_VR ? new GDXVRManager() : null;

   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem();
   private final ImGuiDockingSetup imGuiDockingSetup = new ImGuiDockingSetup();

//   private final GDXLinuxGUIRecorder guiRecorder;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private final Stopwatch runTime = new Stopwatch().start();

   private final ImGui3DViewInput inputCalculator = new ImGui3DViewInput();
   private final ArrayList<Consumer<ImGui3DViewInput>> imGuiInputProcessors = new ArrayList<>();
   private boolean dragging = false;
   private float dragBucketX;
   private float dragBucketY;
   private GLFrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;

   public GDXImGuiBasedUI()
   {
      this("");
   }

   public GDXImGuiBasedUI(String windowTitle)
   {
      this.windowTitle = windowTitle;
//      guiRecorder = new GDXLinuxGUIRecorder(24, 0.8f, getClass().getSimpleName());
//      onCloseRequestListeners.add(guiRecorder::stop);
//      Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

      if (RECORD_VIDEO)
      {
         //         ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
//         ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
      }

      imGuiDockingSetup.addFirst(VIEW_3D_WINDOW_NAME);
   }

   public void create()
   {
      LogTools.info("create()");

      sceneManager.create(GDXInputMode.ImGui);
      if (ENABLE_VR)
         vrManager.create();

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?
      imGuiInputProcessors.add(sceneManager.getCamera3D()::processImGuiInput);

      double isoZoomOut = 7.0;
      sceneManager.getCamera3D().changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);

      sceneManager.addCoordinateFrame(0.3);

      if (ENABLE_VR)
         sceneManager.addRenderableProvider(vrManager, GDXSceneLevel.VIRTUAL);

      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle(), windowTitle);
   }

   public ImGuiDockingSetup getImGuiDockingSetup()
   {
      return imGuiDockingSetup;
   }

   public void pollVREvents()
   {
      if (ENABLE_VR)
      {
         vrManager.pollEvents();
      }
   }

   public void renderBeforeOnScreenUI()
   {
      Gdx.graphics.setTitle(windowTitle + " - " + Gdx.graphics.getFramesPerSecond() + " FPS");
      GDX3DSceneTools.glClearGray(0.3f);
      imGuiWindowAndDockSystem.beforeWindowManagement();

      for (ImGuiWindow window : imGuiDockingSetup.getWindows())
      {
         if (window.isTogglable() && window.getEnabled().get())
         {
            window.render();
         }
      }
   }

   public void renderEnd()
   {
      ImGui.beginMainMenuBar();
      if (ImGui.beginMenu("Window"))
      {
         for (ImGuiWindow window : imGuiDockingSetup.getWindows())
         {
            if (window.isTogglable())
            {
               ImGui.menuItem(window.getWindowName(), "", window.getEnabled());
            }
         }
         if (ImGui.button("Save Layout"))
         {
            saveImGuiSettings();
         }
         ImGui.endMenu();
      }
      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");
      ImGui.endMainMenuBar();

      int flags = ImGuiWindowFlags.None;
       flags |= ImGuiWindowFlags.NoDecoration;
//       flags += ImGuiWindowFlags.MenuBar;
//      flags += ImGuiWindowFlags.NoTitleBar;
//      flags += ImGuiWindowFlags.NoMouseInputs;
//      ImGui.begin(VIEW_3D_WINDOW_NAME, flags);
      ImGui.begin(VIEW_3D_WINDOW_NAME);

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

      if (imGuiWindowAndDockSystem.isFirstRenderCall() && !Files.exists(imGuiWindowAndDockSystem.getImGuiSettingsPath()))
      {
         imGuiDockingSetup.build(imGuiWindowAndDockSystem.getCentralDockspaceId());
         saveImGuiSettings();
      }

      imGuiWindowAndDockSystem.afterWindowManagement();

      if (ENABLE_VR)
         vrManager.render(sceneManager);
   }

   private void saveImGuiSettings()
   {
      String settingsPath = imGuiWindowAndDockSystem.getImGuiSettingsPath().toString();
      LogTools.info("Saving ImGui settings to {}", settingsPath);
      ImGui.saveIniSettingsToDisk(settingsPath);
   }

   public void dispose()
   {
      imGuiWindowAndDockSystem.dispose();
      if (ENABLE_VR)
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

   public GDX3DSceneManager getSceneManager()
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

   public String get3DWindowName()
   {
      return VIEW_3D_WINDOW_NAME;
   }

   public GDXVRManager getVRManager()
   {
      return vrManager;
   }
}
