package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMultiplexer;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   private GDXInputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<GDXRenderable> renderables = new HashSet<>();

   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;

   public void create()
   {
      create(GDX3DSceneTools.createDefaultEnvironment());
   }

   public void create(Environment environment)
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      this.environment = environment;

      DefaultShader.Config defaultShaderConfig = new DefaultShader.Config();
      // we could set shader options or even swap out the shader here
      modelBatch = new ModelBatch(new DefaultShaderProvider(defaultShaderConfig));

      inputMultiplexer = new GDXInputMultiplexer();
      Gdx.input.setInputProcessor(inputMultiplexer);

      camera3D = new FocusBasedGDXCamera();
      inputMultiplexer.addProcessor(camera3D.getInputAdapter());

      if (addFocusSphere)
         addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);

      GDX3DSceneTools.glClearGray();
      Gdx.gl.glEnable(GL32.GL_TEXTURE_2D);
   }

   public void renderBefore()
   {
      renderBefore(GDXSceneLevel.VIRTUAL);
   }

   public void renderBefore(GDXSceneLevel sceneLevel)
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

      modelBatch.begin(camera3D);

      Gdx.gl.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray();

      renderRegisteredObjectsWithEnvironment(modelBatch, sceneLevel);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch)
   {
      renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      for (GDXRenderable renderable : renderables)
      {
         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
         {
            modelBatch.render(renderable.getRenderableProvider(), environment);
         }
      }
   }

   public void renderAfter()
   {
      modelBatch.end();
   }

   public void renderToCamera(Camera camera)
   {
      modelBatch.begin(camera);
      renderRegisteredObjectsWithEnvironment(modelBatch);
      renderAfter();
   }

   public void render()
   {
      render(GDXSceneLevel.VIRTUAL);
   }

   public void render(GDXSceneLevel sceneLevel)
   {
      renderBefore(sceneLevel);
      renderAfter();
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

   public GDXInputMultiplexer getInputMultiplexer()
   {
      return inputMultiplexer;
   }

   public ModelBatch getModelBatch()
   {
      return modelBatch;
   }

   public Environment getEnvironment()
   {
      return environment;
   }

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }
}
