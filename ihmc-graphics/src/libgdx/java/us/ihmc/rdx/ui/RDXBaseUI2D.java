package us.ihmc.rdx.ui;

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
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.imgui.RDXPanelManager;
import us.ihmc.rdx.imgui.RDXPanelSizeHandler;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.input.ImGui2DViewInput;
import us.ihmc.rdx.sceneManager.RDX2DSceneManager;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.*;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class RDXBaseUI2D
{
   public static final int ANTI_ALIASING = 2;

   private static final String VIEW_2D_WINDOW_NAME = "2D View";

   private final RDX2DSceneManager sceneManager = new RDX2DSceneManager();
   private final RDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridResourceDirectory configurationBaseDirectory;
   private HybridResourceFile libGDXSettingsFile;
   private final Stopwatch runTime = new Stopwatch().start();
   private final RDXPanelSizeHandler view2DPanelSizeHandler = new RDXPanelSizeHandler();
   private ImGui2DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui2DViewInput>> imgui2DViewInputProcessors = new ArrayList<>();
   private GLFrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;
   private final ImInt foregroundFPS = new ImInt(240);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImInt libGDXLogLevel = new ImInt(LibGDXTools.toLibGDX(LogTools.getLevel()));
   private final RDXImGuiLayoutManager layoutManager;

   public RDXBaseUI2D(Class<?> classForLoading)
   {
      this(classForLoading, classForLoading.getSimpleName());
   }

   public RDXBaseUI2D(Class<?> classForLoading, String windowTitle)
   {
      this.windowTitle = windowTitle;

      configurationExtraPath = "configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridResourceDirectory(dotIHMCDirectory, classForLoading).resolve(configurationExtraPath);

      layoutManager = new RDXImGuiLayoutManager(classForLoading, configurationExtraPath, configurationBaseDirectory);
      imGuiWindowAndDockSystem = new RDXImGuiWindowAndDockSystem(layoutManager);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      layoutManager.getLayoutDirectoryUpdatedListeners().add(updatedLayoutDirectory ->
      {
         libGDXSettingsFile = new HybridResourceFile(updatedLayoutDirectory, "GDXSettings.json");
      });
      layoutManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      layoutManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         Gdx.graphics.setWindowedMode(imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getWidth(),
                                      imGuiWindowAndDockSystem.getCalculatedPrimaryWindowSize().getHeight());
         ((Lwjgl3Graphics) Gdx.graphics).getWindow().setPosition(imGuiWindowAndDockSystem.getPrimaryWindowContentAreaPosition().getX(),
                                                                 imGuiWindowAndDockSystem.getPrimaryWindowContentAreaPosition().getY());
         return true;
      });
      layoutManager.getSaveListeners().add(this::saveApplicationSettings);
      layoutManager.applyLayoutDirectory();

      imGuiWindowAndDockSystem.getPanelManager().addSelfManagedPanel(VIEW_2D_WINDOW_NAME);
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
      LibGDXApplicationCreator.launchGDXApplication(applicationAdapter, windowTitle, windowWidth.get(), windowHeight.get());
   }

   public void create()
   {
      LogTools.info("Creating...");
      LibGDXTools.printGLVersion();

      sceneManager.create(RDXInputMode.ImGui);
      inputCalculator = new ImGui2DViewInput(sceneManager.getOrthographicCamera(), this::getViewportSizeX, this::getViewportSizeY);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?
      imgui2DViewInputProcessors.add(sceneManager.getOrthographicCamera()::processImGuiInput);



      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));
   }

   public void renderBeforeOnScreenUI()
   {
      Gdx.graphics.setTitle(windowTitle + " - " + Gdx.graphics.getFramesPerSecond() + " FPS");
      imGuiWindowAndDockSystem.beforeWindowManagement();
      render2DView();
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();
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
      ImGui.endMainMenuBar();
   }

   private void render2DView()
   {
      view2DPanelSizeHandler.handleSizeBeforeBegin();
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
      int flags = ImGuiWindowFlags.None;
      ImGui.begin(VIEW_2D_WINDOW_NAME, flags);
      view2DPanelSizeHandler.handleSizeAfterBegin();

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
      sizeX = ImGui.getWindowSizeX();
      sizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
      float renderSizeX = sizeX * ANTI_ALIASING;
      float renderSizeY = sizeY * ANTI_ALIASING;

      inputCalculator.compute();
      for (Consumer<ImGui2DViewInput> imGuiInputProcessor : imgui2DViewInputProcessors)
      {
         imGuiInputProcessor.accept(inputCalculator);
      }

      // Allows for dynamically resizing the 2D view panel. Grows by 2x when needed, but never shrinks.
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

      sceneManager.setViewportBounds((int) renderSizeX, (int) renderSizeY);

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
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   public void addImGui2DViewInputProcessor(Consumer<ImGui2DViewInput> processImGuiInput)
   {
      imgui2DViewInputProcessors.add(processImGuiInput);
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

   public RDXPanelManager getImGuiPanelManager()
   {
      return imGuiWindowAndDockSystem.getPanelManager();
   }

   public RDXImGuiLayoutManager getLayoutManager()
   {
      return layoutManager;
   }

   public RDX2DSceneManager get2DSceneManager()
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

   public RDXImGuiWindowAndDockSystem getImGuiWindowAndDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }

   public HybridDirectory getConfigurationBaseDirectory()
   {
      return configurationBaseDirectory;
   }
}
