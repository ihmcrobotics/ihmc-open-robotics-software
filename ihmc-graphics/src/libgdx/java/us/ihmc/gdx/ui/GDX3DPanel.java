package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiPanelSizeHandler;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.Consumer;

public class GDX3DPanel
{
   private final ImGuiPanelSizeHandler view3DPanelSizeHandler = new ImGuiPanelSizeHandler();
   private final String panelName;
   private final int antiAliasing;
   private ImGuiPanel imGuiPanel;
   private GDX3DScene scene;
   private boolean modelSceneMouseCollisionEnabled = false;
   private GLProfiler glProfiler;
   private SensorFrameBuffer frameBuffer;
   private float windowSizeX;
   private float windowSizeY;
   private ImGui3DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewPickCalculators = new ArrayList<>();
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewInputProcessors = new ArrayList<>();
   private final GDX3DPanelToolbar toolbar = new GDX3DPanelToolbar();
   private final ArrayList<Runnable> imGuiOverlayAdditions = new ArrayList<>();
   private InputMultiplexer inputMultiplexer;
   private GDXFocusBasedCamera camera3D;
   private ScreenViewport viewport;
   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere;
   private Runnable backgroundRenderer;
   private float backgroundShade = GDX3DSceneTools.CLEAR_COLOR;
   private ByteBuffer normalizedDeviceCoordinateDepthDirectByteBuffer;
   private float renderSizeX;
   private float renderSizeY;
   private float windowDrawMinX;
   private float windowDrawMinY;
   private float windowDrawMaxX;
   private float windowDrawMaxY;

   public GDX3DPanel(String panelName, int antiAliasing, boolean addFocusSphere)
   {
      this.panelName = panelName;
      this.antiAliasing = antiAliasing;
      this.addFocusSphere = addFocusSphere;
   }

   public void create(GDXInputMode inputMode, GLProfiler glProfiler, GDX3DScene scene)
   {
      this.glProfiler = glProfiler;
      this.scene = scene;

      imGuiPanel = new ImGuiPanel(panelName, null, false);

      camera3D = new GDXFocusBasedCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }
      inputCalculator = new ImGui3DViewInput(this);
      imgui3DViewInputProcessors.add(camera3D::processImGuiInput);

      if (addFocusSphere)
         scene.addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?
   }

   public void render()
   {
      // NOTE: show panel(window) here
      ImBoolean isShowing = imGuiPanel.getIsShowing();
      if (imGuiPanel.getIsShowing().get())
      {
         view3DPanelSizeHandler.handleSizeBeforeBegin();
         ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
         int flags = ImGuiWindowFlags.None;
         ImGui.begin(panelName, flags);
         view3DPanelSizeHandler.handleSizeAfterBegin();

         float windowPositionX = ImGui.getWindowPosX();
         float windowPositionY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
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
         if (modelSceneMouseCollisionEnabled && scene.getSceneLevelsToRender().contains(GDXSceneLevel.MODEL))
         {
            frameBuffer.begin();
            renderScene(GDXSceneLevel.MODEL.SINGLETON_SET);

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
         ImGui.popStyleVar();

         for (Runnable imguiOverlayAddition : imGuiOverlayAdditions)
         {
            imguiOverlayAddition.run();
         }

         if (ImGui.isMouseDoubleClicked(ImGuiMouseButton.Right))
         {
            camera3D.setCameraFocusPoint(inputCalculator.getPickPointInWorld());
         }

         ImGui.end();

         toolbar.render(windowSizeX, windowPositionX, windowPositionY);
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

   private void renderScene(Set<GDXSceneLevel> sceneLevels)
   {
      preRender();

      if (backgroundRenderer != null)
         backgroundRenderer.run();

      scene.render(sceneLevels);
      scene.postRender(camera3D, sceneLevels);

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
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
      GDX3DSceneTools.glClearGray(backgroundShade);
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

   public GDXFocusBasedCamera getCamera3D()
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

   public GDX3DPanelToolbarButton addToolbarButton()
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

   public GDX3DScene getScene()
   {
      return scene;
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
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
}
