package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DApplication extends Lwjgl3ApplicationAdapter
{
   public static final float CLEAR_COLOR = 0.5019608f;
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<RenderableProvider> renderableProviders = new HashSet<>();
   private final ArrayList<Runnable> preRenderTasks = new ArrayList<>();

   private boolean firstRenderStarted = false;

   @Override
   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      environment = new Environment();
      float ambientColor = 0.7f;
      float pointColor = 0.07f;
      float pointDistance = 2.0f;
      float pointIntensity = 1.0f;
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      // Point lights not working; not sure why @dcalvert
      //      PointLightsAttribute pointLights = new PointLightsAttribute();
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      environment.set(pointLights);
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, -pointDistance));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      inputMultiplexer = new InputMultiplexer();
      Gdx.input.setInputProcessor(inputMultiplexer);

      camera3D = new FocusBasedGDXCamera();
      addModelInstance(camera3D.getFocusPointSphere());
      inputMultiplexer.addProcessor(camera3D.getInputProcessor());
      viewport = new ScreenViewport(camera3D);

      glClearGrayscale();
      Gdx.gl.glEnable(GL32.GL_TEXTURE_2D);
   }

   @Override
   public void resize(int width, int height)
   {
   }

   public void renderBefore()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      for (Runnable preRenderTask : preRenderTasks)
      {
         preRenderTask.run();
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      modelBatch.begin(camera3D);

      Gdx.gl.glViewport(x, y, width, height);

      renderRegisteredObjectsWithEnvironment(modelBatch);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch)
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         modelBatch.render(modelInstance, environment);
      }
      for (RenderableProvider renderableProvider : renderableProviders)
      {
         modelBatch.render(renderableProvider, environment);
      }
   }

   public void renderAfter()
   {
      modelBatch.end();
   }

   public void renderVRCamera(Camera camera)
   {
      modelBatch.begin(camera);
      renderRegisteredObjectsWithEnvironment(modelBatch);
      renderAfter();
   }

   @Override
   public void render()
   {
      renderBefore();
      renderAfter();
   }

   @Override
   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      modelBatch.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public void addModelInstance(ModelInstance modelInstance)
   {
      modelInstances.add(modelInstance);
   }

   public void addCoordinateFrame(double size)
   {
      addModelInstance(GDXModelPrimitives.createCoordinateFrameInstance(size));
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      renderableProviders.add(renderableProvider);
   }

   public void addInputProcessor(InputProcessor inputProcessor)
   {
      inputMultiplexer.addProcessor(inputProcessor);
   }

   public void addPreRenderTask(Runnable task)
   {
      preRenderTasks.add(task);
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

      camera3D.setInputBounds(x, x + width, getCurrentWindowHeight() - y - height, getCurrentWindowHeight() - y);
   }

   public void glClearGrayscale()
   {
      glClearGrayscale(CLEAR_COLOR);
   }

   public void glClearGrayscale(float color)
   {
      Gdx.gl.glClearColor(color, color, color, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);
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

   public ModelBatch getModelBatch()
   {
      return modelBatch;
   }

   public Environment getEnvironment()
   {
      return environment;
   }
}
