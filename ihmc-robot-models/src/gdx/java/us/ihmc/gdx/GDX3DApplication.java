package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DApplication extends Lwjgl3ApplicationAdapter
{
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;

   private double percentXOffset = 0.0;
   private double percentYOffset = 0.0;
   private double percentWide = 1.0;
   private double percentTall = 1.0;

   private HashSet<ModelInstance> modelInstances = new HashSet<>();
   private HashSet<RenderableProvider> renderableProviders = new HashSet<>();

   private ArrayList<Runnable> preRenderTasks = new ArrayList<>();

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

   public void setViewportBounds(double percentXOffset, double percentYOffset, double percentWide, double percentTall)
   {
      this.percentXOffset = percentXOffset;
      this.percentYOffset = percentYOffset;
      this.percentWide = percentWide;
      this.percentTall = percentTall;
   }

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

      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glEnable(GL20.GL_TEXTURE_2D);
   }

   @Override
   public void resize(int width, int height)
   {
   }

   public void renderBefore()
   {
      for (Runnable preRenderTask : preRenderTasks)
      {
         preRenderTask.run();
      }

      viewport.update((int) (Gdx.graphics.getWidth() * percentWide), (int) (Gdx.graphics.getHeight() * percentTall));

      modelBatch.begin(camera3D);

      Gdx.gl.glViewport((int) (Gdx.graphics.getWidth() * percentXOffset),
                        (int) (Gdx.graphics.getHeight() * percentYOffset),
                        (int) (Gdx.graphics.getWidth() * percentWide),
                        (int) (Gdx.graphics.getHeight() * percentTall));

      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      renderRegisteredObjects();
   }

   private void renderRegisteredObjects()
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
      renderRegisteredObjects();
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
         ExceptionTools.handle(() -> modelInstance.model.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      }

      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      modelBatch.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
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
