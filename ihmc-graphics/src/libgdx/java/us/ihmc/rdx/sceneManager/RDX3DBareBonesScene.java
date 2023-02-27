package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.log.LogTools;

/**
 * TODO: Pause and resume?
 */
public class RDX3DBareBonesScene
{
   private final RDX3DScene scene = new RDX3DScene();

   private InputMultiplexer inputMultiplexer;
   private RDXFocusBasedCamera camera3D;
   private ScreenViewport viewport;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;
   private Runnable onCreate;
   private GLProfiler glProfiler;

   public void create()
   {
      create(RDXInputMode.libGDX, RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXInputMode inputMode, RDXSceneLevel... sceneLevels)
   {
      if (LibGDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = LibGDXTools.createGLProfiler();

      LibGDXTools.syncLogLevelWithLogTools();

      camera3D = new RDXFocusBasedCamera();
      if (inputMode == RDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      scene.create(sceneLevels);

      if (addFocusSphere)
         scene.addModelInstance(camera3D.getFocusPointSphere(), RDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      scene.addDefaultLighting();
      if (onCreate != null)
         onCreate.run();
   }

   public void renderShadowMap()
   {
      renderShadowMap(width, height);
   }

   public void renderShadowMap(int x, int y)
   {
      scene.renderShadowMap(camera3D, x, y);
   }

   public void render()
   {
      preRender();
      scene.render();
      scene.postRender(camera3D, RDXSceneLevel.VIRTUAL.SINGLETON_SET);

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
      RDX3DSceneTools.glClearGray();
   }

   public void dispose()
   {
      scene.dispose();
      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
   }
   // End render public API

   public boolean closeRequested()
   {
      return true;
   }

   public void addModelInstance(ModelInstance modelInstance)
   {
      scene.addModelInstance(modelInstance);
   }

   public void addModelInstance(ModelInstance modelInstance, RDXSceneLevel sceneLevel)
   {
      scene.addModelInstance(modelInstance, sceneLevel);
   }

   public void addCoordinateFrame(double size)
   {
      scene.addCoordinateFrame(size);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      scene.addRenderableProvider(renderableProvider);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      scene.addRenderableProvider(renderableProvider, sceneLevel);
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

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }

   public void setOnCreate(Runnable onCreate)
   {
      this.onCreate = onCreate;
   }

   public RDX3DScene getScene()
   {
      return scene;
   }
}
