package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL30;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.lighting.GDXPointLight;
import us.ihmc.gdx.lighting.GDXSceneShader;
import us.ihmc.gdx.lighting.GDXShadowManager;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;

import java.nio.IntBuffer;
import java.util.HashSet;
import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<GDXRenderable> renderables = new HashSet<>();
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private ScreenViewport viewport;
   private ShaderProgram mainShaderProgram;
   private ModelBatch modelBatch;
   private ModelBatch virtualBatch;
   private GDXShadowManager shadowManager;
   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;

   private static int getFramebufferID()
   {
      IntBuffer buffer = BufferUtils.newIntBuffer(1);
      Gdx.gl.glGetIntegerv(GL30.GL_DRAW_FRAMEBUFFER_BINDING, buffer);
      return buffer.get();
   }

   public void create()
   {
      create(GDXInputMode.libGDX);
   }

   public void create(GDXInputMode inputMode)
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      camera3D = new FocusBasedGDXCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      if (addFocusSphere)
         addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      mainShaderProgram = new ShaderProgram(GDXShadowManager.getVertexShader(), GDXShadowManager.getFragmentShader());
      modelBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(Renderable renderable)
         {
            return new GDXSceneShader(renderable, mainShaderProgram);
         }
      });

      virtualBatch = new ModelBatch();

      shadowManager = new GDXShadowManager(GDXImGuiBasedUI.ANTI_ALIASING);

      shadowManager.addLight(new GDXPointLight(new Vector3(5, 10, 5)));
//      shadowManager.addLight(new GDXPointLight(new Vector3(0, 10, 5)));
//      shadowManager.addLight(new GDXPointLight(new Vector3(-5, 10, 5)));
//      shadowManager.addLight(new GDXPointLight(new Vector3(5, 10, -5)));
      //      shadowManager.addLight(new GDXPointLight(new Vector3(0, 10, -5)));
      //      shadowManager.addLight(new GDXPointLight(new Vector3(-5, 10, -5)));

      shadowManager.update();
   }

   public void renderShadowMap(int x, int y)
   {
      shadowManager.renderShadows(camera3D, new GDXRenderableIterable(renderables), x, y);
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
      shadowManager.apply(mainShaderProgram);
      modelBatch.begin(camera3D);
      Gdx.gl.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray();
   }

   private void renderInternal(ModelBatch modelBatch)
   {
      renderInternal(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   private void renderInternal(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      // All rendering except modelBatch.begin() and end()

      for (GDXRenderable renderable : renderables)
      {
         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
            modelBatch.render(renderable);
      }
   }

   private void postRender()
   {
      modelBatch.end(); // This is actually where all the rendering happens despite the method name

      //Render virtual objects to the screen too
      virtualBatch.begin(camera3D);
      virtualBatch.render(new GDXRenderableIterable(renderables, GDXSceneLevel.VIRTUAL));
      virtualBatch.end();
   }

   // Render public API
   public void renderToCamera(Camera camera)
   {
      modelBatch.begin(camera);
      renderInternal(modelBatch);
      postRender();
   }

   public void render()
   {
      render(GDXSceneLevel.VIRTUAL);
   }

   public void render(GDXSceneLevel sceneLevel)
   {
      preRender();
      renderInternal(modelBatch, sceneLevel);
      postRender();
   }

   public void renderFromBatch(ModelBatch batch, GDXSceneLevel sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      modelBatch.dispose();
   }
   // End render public API

   public boolean closeRequested()
   {
      return true;
   }

   public void addModelInstance(ModelInstance modelInstance)
   {
      addModelInstance(modelInstance, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void addModelInstance(ModelInstance modelInstance, GDXSceneLevel sceneLevel)
   {
      addRenderableProvider(modelInstance, sceneLevel);
      modelInstances.add(modelInstance);
   }

   public void addCoordinateFrame(double size)
   {
      addModelInstance(GDXModelPrimitives.createCoordinateFrameInstance(size), GDXSceneLevel.VIRTUAL);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      addRenderableProvider(renderableProvider, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      renderables.add(new GDXRenderable(renderableProvider, sceneLevel));
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

      this.shadowManager.setViewportBounds(x, y, width, height);
   }

   public int getCurrentWindowWidth()
   {
      return Gdx.graphics.getWidth();
   }

   public int getCurrentWindowHeight()
   {
      return Gdx.graphics.getHeight();
   }

   public FocusBasedGDXCamera getCamera3D()
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

   private class GDXRenderableIterable implements Iterable<GDXRenderable>
   {
      private final HashSet<GDXRenderable> renderables;
      private final GDXSceneLevel level;

      protected GDXRenderableIterable(HashSet<GDXRenderable> renderables)
      {
         this(renderables, GDXSceneLevel.REAL_ENVIRONMENT);
      }

      protected GDXRenderableIterable(HashSet<GDXRenderable> renderables, GDXSceneLevel level)
      {
         this.renderables = renderables;
         this.level = level;
      }

      @Override
      public Iterator<GDXRenderable> iterator()
      {
         return new GDXRenderableIterator();
      }

      private class GDXRenderableIterator implements Iterator<GDXRenderable>
      {
         private final HashSet<GDXRenderable> renderablesInternal;

         private GDXRenderableIterator()
         {
            renderablesInternal = new HashSet<>(renderables);
         }

         @Override
         public boolean hasNext()
         {
            for (GDXRenderable renderable : renderablesInternal)
            {
               if (renderable.getSceneType() == level)
               {
                  return true;
               }
            }

            return false;
         }

         @Override
         public GDXRenderable next()
         {
            Iterator<GDXRenderable> it = renderablesInternal.iterator();
            while (it.hasNext())
            {
               GDXRenderable renderable = it.next();
               it.remove();

               if (renderable.getSceneType() == level)
               {
                  return renderable;
               }
            }

            throw new NoSuchElementException();
         }
      }
   }
}
