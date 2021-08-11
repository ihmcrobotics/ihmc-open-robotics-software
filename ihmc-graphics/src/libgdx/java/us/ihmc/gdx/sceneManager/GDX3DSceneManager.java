package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL30;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.lighting.*;
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
   private ShaderProgram shadowSceneShaderProgram;

   private ModelBatch shadowObjectsModelBatch;
   private ModelBatch primaryModelBatch;

   private GDXShadowManager shadowManager;
   private ModelBatch currentRenderingBatch;

   private boolean shadowsEnabled = false;
   private Environment shadowsDisabledEnvironment;
   private int shadowsHashCode = 0;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;
   private float pointLightIntensity = 1.0f; //THIS VALUE SHOULD BE MOVED SOMEWHERE ELSE

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

      shadowSceneShaderProgram = new ShaderProgram(GDXShadowManager.getVertexShader(), GDXShadowManager.getFragmentShader());
      shadowObjectsModelBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(Renderable renderable)
         {
            return new GDXSceneShader(renderable, shadowSceneShaderProgram);
         }
      });

      primaryModelBatch = new ModelBatch();

      shadowsDisabledEnvironment = new Environment();
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(0.4f, 0.4f, 0.4f, 1.0f));

      shadowManager = new GDXShadowManager(GDXImGuiBasedUI.ANTI_ALIASING);
      shadowManager.update();
   }

   public void renderShadowMap()
   {
      renderShadowMap(width, height);
   }

   public void renderShadowMap(int x, int y)
   {
      shadowManager.renderShadows(camera3D, new GDXRenderableIterable(renderables), x, y);
   }

   /**
    * Updates LIGHTS for the environment - ambient light is handled separately
    */
   private void updateEnvironment()
   {
      shadowsHashCode = shadowManager.getLights().hashCode();
      shadowsDisabledEnvironment.clear();
      for (GDXLight light : shadowManager.getLights())
      {
         if (light instanceof GDXPointLight)
         {
            shadowsDisabledEnvironment.add(new PointLight().set(Color.WHITE, light.getPosition(), pointLightIntensity));
         }
         else if (light instanceof GDXDirectionalLight)
         {
            shadowsDisabledEnvironment.add(new DirectionalLight().set(Color.WHITE, ((GDXDirectionalLight) light).getDirection()));
         }
      }
   }

   private void preRender()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");

         updateEnvironment();
      }

      if (shadowsHashCode != shadowManager.getLights().hashCode())
      {
         updateEnvironment();
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      if (shadowsEnabled)
      {
         shadowManager.apply(shadowSceneShaderProgram);
         currentRenderingBatch = shadowObjectsModelBatch;
      }
      else
      {
         currentRenderingBatch = primaryModelBatch;

         if (shadowManager != null)
         {
            float intensity = shadowManager.getAmbientLight();
            shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(intensity, intensity, intensity, 1.0f));
         }
      }

      currentRenderingBatch.begin(camera3D);

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
         {
            if (shadowsEnabled)
               modelBatch.render(renderable);
            else
               modelBatch.render(renderable, shadowsDisabledEnvironment);
         }
      }
   }

   private void postRender()
   {
      currentRenderingBatch.end(); // This is actually where all the rendering happens despite the method name
      currentRenderingBatch = null;

      //Render all virtual objects using the primary model batch
      primaryModelBatch.begin(camera3D);
      primaryModelBatch.render(new GDXRenderableIterable(renderables, GDXSceneLevel.VIRTUAL));
      primaryModelBatch.end();
   }

   // Render public API
   public void renderToCamera(Camera camera)
   {
      if (shadowsEnabled)
      {
         currentRenderingBatch = shadowObjectsModelBatch;
      }
      else
      {
         currentRenderingBatch = primaryModelBatch;
      }

      currentRenderingBatch.begin(camera);
      renderInternal(currentRenderingBatch);
      postRender();
   }

   public void render()
   {
      render(GDXSceneLevel.VIRTUAL);
   }

   public void render(GDXSceneLevel sceneLevel)
   {
      preRender();
      renderInternal(currentRenderingBatch, sceneLevel);
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
      shadowObjectsModelBatch.dispose();
      primaryModelBatch.dispose();
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

   public GDXShadowManager getShadowManager()
   {
      return shadowManager;
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

   public void setShadowsEnabled(boolean shadowsEnabled)
   {
      this.shadowsEnabled = shadowsEnabled;
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
