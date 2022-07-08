package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
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

import java.util.ArrayList;
import java.util.function.Consumer;

public class GDX3DPanel
{
   private final ImGuiPanelSizeHandler view3DPanelSizeHandler = new ImGuiPanelSizeHandler();
   private final String panelName;
   private final int antiAliasing;
   private ImGuiPanel imGuiPanel;
   private GDX3DScene scene;
   private GLProfiler glProfiler;
   private FrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;
   private ImGui3DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewPickCalculators = new ArrayList<>();
   private final ArrayList<Consumer<ImGui3DViewInput>> imgui3DViewInputProcessors = new ArrayList<>();
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
      inputCalculator = new ImGui3DViewInput(camera3D, this::getViewportSizeX, this::getViewportSizeY);
      imgui3DViewInputProcessors.add(camera3D::processImGuiInput);

      if (addFocusSphere)
         scene.addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?
   }

   public void render()
   {
      if (imGuiPanel.getIsShowing().get())
      {
         view3DPanelSizeHandler.handleSizeBeforeBegin();
         ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
         int flags = ImGuiWindowFlags.None;
         ImGui.begin(panelName, flags);
         view3DPanelSizeHandler.handleSizeAfterBegin();

         float posX = ImGui.getWindowPosX();
         float posY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
         sizeX = ImGui.getWindowSizeX();
         sizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
         float renderSizeX = sizeX * antiAliasing;
         float renderSizeY = sizeY * antiAliasing;

         inputCalculator.compute();
         for (Consumer<ImGui3DViewInput> imgui3DViewPickCalculator : imgui3DViewPickCalculators)
         {
            imgui3DViewPickCalculator.accept(inputCalculator);
         }
         inputCalculator.calculateClosestPick();
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
            GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(newWidth, newHeight);
            frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
            frameBufferBuilder.addBasicStencilDepthPackedRenderBuffer();
            frameBuffer = frameBufferBuilder.build();
         }

         setViewportBounds(0, 0, (int) renderSizeX, (int) renderSizeY);
         renderShadowMap(Gdx.graphics.getWidth() * antiAliasing, Gdx.graphics.getHeight() * antiAliasing);

         frameBuffer.begin();
         renderScene();
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

         frameBuffer.begin();
         renderScene();
         frameBuffer.end();

         ImGui.getWindowDrawList().addImage(textureID, pMinX, pMinY, pMaxX, pMaxY, uvMinX, uvMinY, uvMaxX, uvMaxY);

         ImGui.end();
         ImGui.popStyleVar();
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

   private void renderScene()
   {
      preRender();

      if (backgroundRenderer != null)
         backgroundRenderer.run();

      scene.render();
      scene.postRender(camera3D, GDXSceneLevel.VIRTUAL);

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
      GDX3DSceneTools.glClearGray();
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
      return sizeX;
   }

   public float getViewportSizeY()
   {
      return sizeY;
   }

   public void addImGui3DViewPickCalculator(Consumer<ImGui3DViewInput> calculate3DViewPick)
   {
      imgui3DViewPickCalculators.add(calculate3DViewPick);
   }

   public void addImGui3DViewInputProcessor(Consumer<ImGui3DViewInput> processImGuiInput)
   {
      imgui3DViewInputProcessors.add(processImGuiInput);
   }

   public GDX3DScene getScene()
   {
      return scene;
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }

   public FrameBuffer getFrameBuffer()
   {
      return frameBuffer;
   }

   public int getAntiAliasing()
   {
      return antiAliasing;
   }
}
