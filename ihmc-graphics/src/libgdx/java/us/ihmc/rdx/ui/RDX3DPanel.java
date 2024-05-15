package us.ihmc.rdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.RDXPanelSizeHandler;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Consumer;

public class RDX3DPanel extends RDXPanel
{
   public static final int OVERLAY_BACKGROUND_COLOR = new Color(1.0f, 1.0f, 1.0f, 0.5f).toIntBits();
   private final RDXPanelSizeHandler view3DPanelSizeHandler = new RDXPanelSizeHandler();
   private final String panelName;
   private final int antiAliasing;
   private RDX3DScene scene;
   private boolean modelSceneMouseCollisionEnabled = false;
   private GLProfiler glProfiler;
   private SensorFrameBuffer frameBuffer;
   private float windowSizeX;
   private float windowSizeY;
   private ImGui3DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewPickCalculators = new ArrayList<>();
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewInputProcessors = new ArrayList<>();
   private final Map<Object, Consumer<ImGui3DViewInput>> imgui3DViewPickCalculatorOwnerKeyMap = new HashMap<>();
   private final Map<Object, Consumer<ImGui3DViewInput>> imgui3DViewInputProcessorOwnerKeyMap = new HashMap<>();
   private final RDX3DPanelToolbar toolbar = new RDX3DPanelToolbar();
   private final ArrayList<Runnable> imGuiOverlayAdditions = new ArrayList<>();
   private final TreeMap<String, RDX3DOverlayPanel> overlayPanels = new TreeMap<>();
   private final Map<Object, Runnable> imGuiOverlayAdditionOwnerKeyMap = new HashMap<>();
   private InputMultiplexer inputMultiplexer;
   private RDXFocusBasedCamera camera3D;
   private ScreenViewport viewport;
   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere;
   private Runnable backgroundRenderer;
   private float backgroundShade = RDX3DSceneTools.CLEAR_COLOR;
   private ByteBuffer normalizedDeviceCoordinateDepthDirectByteBuffer;
   private float renderSizeX;
   private float renderSizeY;
   private float windowDrawMinX;
   private float windowDrawMinY;
   private float windowDrawMaxX;
   private float windowDrawMaxY;
   private float windowPositionX;
   private float windowPositionY;
   private final RDX3DPanelNotificationManager notificationManager = new RDX3DPanelNotificationManager(this);

   public RDX3DPanel(String panelName)
   {
      this(panelName, RDXBaseUI.ANTI_ALIASING, true);
   }

   /**
    * @param addFocusSphere show the little red sphere used to tell where the camera focus is
    */
   public RDX3DPanel(String panelName, boolean addFocusSphere)
   {
      this(panelName, RDXBaseUI.ANTI_ALIASING, addFocusSphere);
   }

   /**
    * @param antiAliasing 1, 2, or 4
    * @param addFocusSphere show the little red sphere used to tell where the camera focus is
    */
   public RDX3DPanel(String panelName, int antiAliasing, boolean addFocusSphere)
   {
      super(panelName);
      super.setRenderMethod(null);
      this.panelName = panelName;
      this.antiAliasing = antiAliasing;
      this.addFocusSphere = addFocusSphere;
   }

   public void create(RDXInputMode inputMode, GLProfiler glProfiler, RDX3DScene scene)
   {
      this.glProfiler = glProfiler;
      this.scene = scene;

      camera3D = new RDXFocusBasedCamera();
      if (inputMode == RDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }
      inputCalculator = new ImGui3DViewInput(this);
      imgui3DViewInputProcessors.add(camera3D::processImGuiInput);

      if (addFocusSphere)
         scene.addModelInstance(camera3D.getFocusPointSphere(), RDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      addImGuiOverlayAddition(notificationManager::render);
   }

   public void render()
   {
      if (getIsShowing().get() && ImGuiTools.getCurrentContext() != 0)
      {
         view3DPanelSizeHandler.handleSizeBeforeBegin();
         ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
         int flags = ImGuiWindowFlags.None;
         ImGui.begin(panelName, flags);
         view3DPanelSizeHandler.handleSizeAfterBegin();
         ImGui.popStyleVar();

         windowPositionX = ImGui.getWindowPosX();
         windowPositionY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
         windowSizeX = ImGui.getWindowSizeX();
         windowSizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
         renderSizeX = windowSizeX * antiAliasing;
         renderSizeY = windowSizeY * antiAliasing;

         inputCalculator.compute();
         if (inputCalculator.isWindowHovered()) // If the window is not hovered, we should not be computing picks
         {
            for (Consumer<ImGui3DViewInput> imgui3DViewPickCalculator : imgui3DViewPickCalculators)
            {
               imgui3DViewPickCalculator.accept(inputCalculator);
            }
            inputCalculator.calculateClosestPick();
         }
         for (Consumer<ImGui3DViewInput> imGuiInputProcessor : imgui3DViewInputProcessors)
         {
            imGuiInputProcessor.accept(inputCalculator);
         }

         // Allows for dynamically resizing the 3D view panel. Grows by 2x when needed, but never shrinks.
         if (frameBuffer == null || frameBuffer.getWidth() < renderSizeX || frameBuffer.getHeight() < renderSizeY)
         {
            if (frameBuffer != null)
               frameBuffer.dispose();

            int newWidth = frameBuffer == null ? Gdx.graphics.getWidth() * antiAliasing : frameBuffer.getWidth() * 2;
            int newHeight = frameBuffer == null ? Gdx.graphics.getHeight() * antiAliasing : frameBuffer.getHeight() * 2;
            LogTools.info("Allocating framebuffer of size: {}x{}", newWidth, newHeight);
            SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(newWidth, newHeight);
            frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
            frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
            frameBufferBuilder.addColorTextureAttachment(GL41.GL_R32F, GL41.GL_RED, GL41.GL_FLOAT);
            frameBuffer = frameBufferBuilder.build();

            int bytesPerPixel = Float.BYTES;
            normalizedDeviceCoordinateDepthDirectByteBuffer = ByteBuffer.allocateDirect(newWidth * newHeight * bytesPerPixel);
            normalizedDeviceCoordinateDepthDirectByteBuffer.order(ByteOrder.nativeOrder());
         }

         setViewportBounds(0, 0, (int) renderSizeX, (int) renderSizeY);
         renderShadowMap(Gdx.graphics.getWidth() * antiAliasing, Gdx.graphics.getHeight() * antiAliasing);

         int frameBufferWidth = frameBuffer.getWidth();
         int frameBufferHeight = frameBuffer.getHeight();

         // We do this render to get the Z buffer from just the model
         if (modelSceneMouseCollisionEnabled && scene.getSceneLevelsToRender().contains(RDXSceneLevel.MODEL))
         {
            frameBuffer.begin();
            renderScene(RDXSceneLevel.MODEL.SINGLETON_SET);

            normalizedDeviceCoordinateDepthDirectByteBuffer.rewind(); // SIGSEV otherwise
            GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT1);
            GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read floats
            // Note: This line has significant performance impact
            GL41.glReadPixels(0, 0, (int) renderSizeX, (int) renderSizeY, GL41.GL_RED, GL41.GL_FLOAT, normalizedDeviceCoordinateDepthDirectByteBuffer);
            GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did

            frameBuffer.end();
         }

         // The scene will render twice if both real and virtual environments are showing
         frameBuffer.begin();
         renderScene(scene.getSceneLevelsToRender());
         frameBuffer.end();

         float percentOfFramebufferUsedX = renderSizeX / frameBufferWidth;
         float percentOfFramebufferUsedY = renderSizeY / frameBufferHeight;
         int textureID = frameBuffer.getColorBufferTexture().getTextureObjectHandle();
         windowDrawMinX = windowPositionX;
         windowDrawMinY = windowPositionY;
         windowDrawMaxX = windowPositionX + windowSizeX;
         windowDrawMaxY = windowPositionY + windowSizeY;
         float uvMinX = 0.0f;
         float uvMinY = percentOfFramebufferUsedY; // flip Y
         float uvMaxX = percentOfFramebufferUsedX;
         float uvMaxY = 0.0f;

         ImGui.getWindowDrawList().addImage(textureID, windowDrawMinX, windowDrawMinY, windowDrawMaxX, windowDrawMaxY, uvMinX, uvMinY, uvMaxX, uvMaxY);

         for (Runnable imguiOverlayAddition : imGuiOverlayAdditions)
            imguiOverlayAddition.run();

         // Render overlay panels
         {
            float previousActiveWindowY = (getWindowPositionY() + 10);
            for (String overlayPanelName : overlayPanels.keySet())
            {
               RDX3DOverlayPanel overlayPanel = overlayPanels.get(overlayPanelName);
               previousActiveWindowY = overlayPanel.render(previousActiveWindowY);
            }
         }

         toolbar.render(windowSizeX, windowPositionX, windowPositionY);

         if (ImGui.isWindowHovered() && ImGui.isMouseDoubleClicked(ImGuiMouseButton.Right))
         {
            camera3D.setCameraFocusPoint(inputCalculator.getPickPointInWorld());
         }

         ImGui.end();
      }
   }

   private void renderShadowMap()
   {
      renderShadowMap(width, height);
   }

   private void renderShadowMap(int x, int y)
   {
      scene.renderShadowMap(camera3D, x, y);
   }

   private void renderScene(Set<RDXSceneLevel> sceneLevels)
   {
      preRender();

      if (backgroundRenderer != null)
         backgroundRenderer.run();

      scene.render(sceneLevels);
      scene.postRender(camera3D, sceneLevels);

      if (LibGDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler.reset();
   }

   private void preRender()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      scene.preRender(camera3D);

      GL41.glViewport(x, y, width, height);
      RDX3DSceneTools.glClearGray(backgroundShade);
   }

   public void dispose()
   {
      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public void setViewportBoundsToWindow()
   {
      setViewportBounds(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
   }

   /**
    * Coordinates in xy bottom left
    */
   public void setViewportBounds(int x, int y, int width, int height)
   {
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;

      scene.getShadowManager().setViewportBounds(x, y, width, height);
   }

   public int getCurrentWindowWidth()
   {
      return Gdx.graphics.getWidth();
   }

   public int getCurrentWindowHeight()
   {
      return Gdx.graphics.getHeight();
   }

   public RDXFocusBasedCamera getCamera3D()
   {
      return camera3D;
   }

   public void addLibGDXInputProcessor(InputProcessor inputProcessor)
   {
      if (inputMultiplexer != null)
      {
         inputMultiplexer.addProcessor(inputProcessor);
      }
      else
      {
         LogTools.error(1, "libGDX is not being used for input!");
      }
   }

   public RDX3DPanelToolbarButton addToolbarButton()
   {
      return toolbar.addButton();
   }

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }

   public void setBackgroundRenderer(Runnable backgroundRenderer)
   {
      this.backgroundRenderer = backgroundRenderer;
   }

   public float getViewportSizeX()
   {
      return windowSizeX;
   }

   public float getViewportSizeY()
   {
      return windowSizeY;
   }

   public void addImGui3DViewPickCalculator(Consumer<ImGui3DViewInput> calculate3DViewPick)
   {
      imgui3DViewPickCalculators.add(calculate3DViewPick);
   }

   public void addImGui3DViewInputProcessor(Consumer<ImGui3DViewInput> processImGuiInput)
   {
      imgui3DViewInputProcessors.add(processImGuiInput);
   }

   public void addImGuiOverlayAddition(Runnable imGuiOverlayAddition)
   {
      imGuiOverlayAdditions.add(imGuiOverlayAddition);
   }

   public void addOverlayPanel(String panelName, Runnable imGuiRender)
   {
      RDX3DOverlayPanel panel = new RDX3DOverlayPanel(panelName, imGuiRender, this);
      overlayPanels.put(panelName, panel);
   }

   public void removeOverlayPanel(String panelName)
   {
      overlayPanels.remove(panelName);
   }

   public void addImGui3DViewPickCalculator(Object ownerKey, Consumer<ImGui3DViewInput> calculate3DViewPick)
   {
      imgui3DViewPickCalculatorOwnerKeyMap.put(ownerKey, calculate3DViewPick);
      imgui3DViewPickCalculators.add(calculate3DViewPick);
   }

   public void addImGui3DViewInputProcessor(Object ownerKey, Consumer<ImGui3DViewInput> processImGuiInput)
   {
      imgui3DViewInputProcessorOwnerKeyMap.put(ownerKey, processImGuiInput);
      imgui3DViewInputProcessors.add(processImGuiInput);
   }

   public void addImGuiOverlayAddition(Object ownerKey, Runnable imGuiOverlayAddition)
   {
      imGuiOverlayAdditionOwnerKeyMap.put(ownerKey, imGuiOverlayAddition);
      imGuiOverlayAdditions.add(imGuiOverlayAddition);
   }

   public void removeImGui3DViewPickCalculator(Object ownerKey)
   {
      imgui3DViewPickCalculators.remove(imgui3DViewPickCalculatorOwnerKeyMap.remove(ownerKey));
   }

   public void removeImGui3DViewInputProcessor(Object ownerKey)
   {
      imgui3DViewInputProcessors.remove(imgui3DViewInputProcessorOwnerKeyMap.remove(ownerKey));
   }

   public void removeImGuiOverlayAddition(Object ownerKey)
   {
      Runnable item = imGuiOverlayAdditionOwnerKeyMap.remove(ownerKey);
      imGuiOverlayAdditionOwnerKeyMap.remove(item);
   }

   public RDX3DScene getScene()
   {
      return scene;
   }

   public SensorFrameBuffer getFrameBuffer()
   {
      return frameBuffer;
   }

   public ByteBuffer getNormalizedDeviceCoordinateDepthDirectByteBuffer()
   {
      return normalizedDeviceCoordinateDepthDirectByteBuffer;
   }

   public int getAntiAliasing()
   {
      return antiAliasing;
   }

   public float getRenderSizeX()
   {
      return renderSizeX;
   }

   public float getRenderSizeY()
   {
      return renderSizeY;
   }

   public float getWindowDrawMinX()
   {
      return windowDrawMinX;
   }

   public float getWindowDrawMinY()
   {
      return windowDrawMinY;
   }

   public float getWindowDrawMaxX()
   {
      return windowDrawMaxX;
   }

   public float getWindowDrawMaxY()
   {
      return windowDrawMaxY;
   }

   public void setBackgroundShade(float backgroundShade)
   {
      this.backgroundShade = backgroundShade;
   }

   public void setModelSceneMouseCollisionEnabled(boolean modelSceneMouseCollisionEnabled)
   {
      this.modelSceneMouseCollisionEnabled = modelSceneMouseCollisionEnabled;
   }

   public float getWindowSizeX()
   {
      return windowSizeX;
   }

   public float getWindowPositionX()
   {
      return windowPositionX;
   }

   public float getWindowPositionY()
   {
      return windowPositionY;
   }

   public RDX3DPanelToolbar getToolbar()
   {
      return toolbar;
   }

   public String getPanelName()
   {
      return panelName;
   }

   public RDX3DPanelNotificationManager getNotificationManager()
   {
      return notificationManager;
   }
}